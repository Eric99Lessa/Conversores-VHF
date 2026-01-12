#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_crc.h"

#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK 14
#define PIN_CS 27
#define SPI_HOST VSPI_HOST

// Fixed SPI transfer size (MOSI and MISO) — MUST match slave
#define SPI_XFER_BYTES 64

static const char *TAG = "SPI_MASTER";

// Global telemetry polling period (ms)
static uint32_t g_telemetry_period_ms = 100;

// ===== TELEMETRY FRAME (slave -> master) =====
#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;  // 0xCAFE
    uint8_t version; // 1
    uint8_t reserved0;
    uint32_t seq;

    float x[6];
    uint16_t duty[3]; // 0..1000 permille

    uint32_t crc32;
} telemetry_frame_t;
#pragma pack(pop)

_Static_assert(sizeof(telemetry_frame_t) == (2 + 1 + 1 + 4 + 6 * 4 + 3 * 2 + 4),
               "Unexpected telemetry frame size");

// ===== COMMAND FRAME (master -> slave) =====
typedef enum
{
    CTRL_MODE_DUTY = 0, // user-sent duty cycle
    CTRL_MODE_PI = 1,
    CTRL_MODE_IDA_PBC = 2,
} control_mode_t;

#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;  // 0xBEEF
    uint8_t version; // 1
    uint8_t mode;    // control_mode_t
    uint8_t reserved0;

    uint16_t duty_cmd[3]; // Permille (0..1000), used when mode == CTRL_MODE_DUTY

    uint32_t crc32;
} command_frame_t;
#pragma pack(pop)

_Static_assert(sizeof(command_frame_t) == (2 + 1 + 1 + 1 + 3 * 2 + 4),
               "Unexpected command frame size");

// compile-time protocol checks
_Static_assert(SPI_XFER_BYTES >= sizeof(telemetry_frame_t), "SPI_XFER_BYTES too small for telemetry");
_Static_assert(SPI_XFER_BYTES >= sizeof(command_frame_t), "SPI_XFER_BYTES too small for command");

static spi_device_handle_t spi;

static command_frame_t g_cmd = {
    .magic = 0xBEEF,
    .version = 1,
    .mode = CTRL_MODE_DUTY,
    .reserved0 = 0,
    .duty_cmd = {200, 200, 200},
    .crc32 = 0,
};

static portMUX_TYPE cmd_mux = portMUX_INITIALIZER_UNLOCKED;

// ===== SPI helpers =====
static void spi_master_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_XFER_BYTES,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .flags = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));

    ESP_LOGI(TAG, "SPI master init. XFER=%u bytes Telem=%u Cmd=%u",
             (unsigned)SPI_XFER_BYTES, (unsigned)sizeof(telemetry_frame_t), (unsigned)sizeof(command_frame_t));
}

static void command_finalize_crc(command_frame_t *c)
{
    c->crc32 = 0;
    c->crc32 = esp_crc32_le(0, (const uint8_t *)c, sizeof(*c) - sizeof(c->crc32));
}

static bool telemetry_crc_ok(const telemetry_frame_t *f)
{
    uint32_t saved = f->crc32;

    telemetry_frame_t tmp;
    memcpy(&tmp, f, sizeof(tmp));
    tmp.crc32 = 0;

    uint32_t crc = esp_crc32_le(0, (const uint8_t *)&tmp, sizeof(tmp) - sizeof(tmp.crc32));
    return (crc == saved);
}

// Full-duplex: send command (in first bytes), receive telemetry (in first bytes)
static esp_err_t spi_exchange(telemetry_frame_t *telem_out)
{
    command_frame_t cmd;
    taskENTER_CRITICAL(&cmd_mux);
    cmd = g_cmd;
    taskEXIT_CRITICAL(&cmd_mux);

    uint8_t tx_buf[SPI_XFER_BYTES];
    uint8_t rx_buf[SPI_XFER_BYTES];
    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));

    memcpy(tx_buf, &cmd, sizeof(command_frame_t));

    spi_transaction_t t = {
        .length = 8 * SPI_XFER_BYTES,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t err = spi_device_transmit(spi, &t);
    if (err != ESP_OK)
        return err;

    memcpy(telem_out, rx_buf, sizeof(*telem_out));
    return ESP_OK;
}

// ===== UART Console parsing =====
static int clamp_i(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static bool parse_d_command(const char *line, uint16_t out_duty[3], uint8_t current_mode)
{
    // Accept: "D 500" or "D 200 500 0" (case-insensitive)
    while (*line && isspace((unsigned char)*line))
        line++;

    if (*line != 'D' && *line != 'd')
        return false;
    line++;

    // Check if we are in Serial Duty mode
    if (current_mode != CTRL_MODE_DUTY)
    {
        printf("CMD_ERR: Duty can only be set in SERIAL mode. Use 'M serial' first.\n");
        return false;
    }

    // parse up to 3 integers after D
    int vals[3];
    int n = 0;

    while (n < 3)
    {
        while (*line && isspace((unsigned char)*line))
            line++;
        if (!*line)
            break;

        char *end = NULL;
        long v = strtol(line, &end, 10);
        if (end == line)
        {
            printf("CMD_ERR: Invalid duty format\n");
            return false; // not a number
        }
        vals[n++] = (int)v;
        line = end;
    }

    if (n == 1)
    {
        int d = clamp_i(vals[0], 0, 1000);
        out_duty[0] = out_duty[1] = out_duty[2] = (uint16_t)d;
        return true;
    }
    else if (n == 3)
    {
        out_duty[0] = (uint16_t)clamp_i(vals[0], 0, 1000);
        out_duty[1] = (uint16_t)clamp_i(vals[1], 0, 1000);
        out_duty[2] = (uint16_t)clamp_i(vals[2], 0, 1000);
        return true;
    }

    printf("CMD_ERR: Invalid duty format (use 1 or 3 values)\n");
    return false;
}

static bool parse_m_command(const char *line, control_mode_t *out_mode)
{
    // Accept: "M serial", "M PI", "M IDAPBC" (case-insensitive)
    while (*line && isspace((unsigned char)*line))
        line++;

    if (*line != 'M' && *line != 'm')
        return false;
    line++;

    while (*line && isspace((unsigned char)*line))
        line++;
    if (!*line)
        return false;

    // compare token
    if (strncasecmp(line, "serial", 6) == 0)
    {
        *out_mode = CTRL_MODE_DUTY; // "serial duty"
        return true;
    }
    if (strncasecmp(line, "pi", 2) == 0)
    {
        *out_mode = CTRL_MODE_PI;
        return true;
    }
    if (strncasecmp(line, "idapbc", 6) == 0)
    {
        *out_mode = CTRL_MODE_IDA_PBC;
        return true;
    }

    return false;
}

static void console_uart_task(void *arg)
{
    (void)arg;

    // Install UART driver for UART0 (the default console UART)
    const uart_port_t U = UART_NUM_0;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(U, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(U, 1024, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART console ready. Commands: D <duty> | D <u> <v> <w> | M serial|PI|IDAPBC");

    char line[128];
    int idx = 0;

    while (1)
    {
        uint8_t ch;
        int r = uart_read_bytes(U, &ch, 1, pdMS_TO_TICKS(100));
        if (r <= 0)
            continue;

        // Handle line endings (both \r and \n)
        if (ch == '\r' || ch == '\n')
        {
            if (idx == 0)
                continue; // ignore empty lines

            line[idx] = '\0';
            idx = 0;

            uint16_t duty[3];
            control_mode_t new_mode;

            if (parse_d_command(line, duty, g_cmd.mode))
            {
                taskENTER_CRITICAL(&cmd_mux);
                g_cmd.mode = CTRL_MODE_DUTY;
                g_cmd.duty_cmd[0] = duty[0];
                g_cmd.duty_cmd[1] = duty[1];
                g_cmd.duty_cmd[2] = duty[2];
                command_finalize_crc(&g_cmd);
                taskEXIT_CRITICAL(&cmd_mux);

                ESP_LOGI(TAG, "CMD_OK: DUTY U=%u V=%u W=%u", duty[0], duty[1], duty[2]);
            }
            else if (parse_m_command(line, &new_mode))
            {
                taskENTER_CRITICAL(&cmd_mux);
                g_cmd.mode = (uint8_t)new_mode;
                command_finalize_crc(&g_cmd);
                taskEXIT_CRITICAL(&cmd_mux);

                const char *name = (new_mode == CTRL_MODE_DUTY) ? "SERIAL/DUTY" : (new_mode == CTRL_MODE_PI) ? "PI"
                                                                                                             : "IDAPBC";
                ESP_LOGI(TAG, "CMD_OK: MODE %s", name);
            }
            else
            {
                ESP_LOGW(TAG, "CMD_ERR: Use D <duty> | D <u> <v> <w> | M serial|PI|IDAPBC");
            }
            continue;
        }

        // Basic line editing (backspace)
        if (ch == 0x08 || ch == 0x7F)
        {
            if (idx > 0)
                idx--;
            continue;
        }

        // Accumulate characters
        if (idx < (int)sizeof(line) - 1)
        {
            line[idx++] = (char)ch;
        }
        else
        {
            // Buffer overflow: reset
            ESP_LOGW(TAG, "Line buffer overflow, resetting");
            idx = 0;
        }
    }
}

// ===== Telemetry polling task =====
static void telemetry_task(void *arg)
{
    (void)arg;

    while (1)
    {
        telemetry_frame_t f;
        esp_err_t err = spi_exchange(&f);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "SPI exchange failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(g_telemetry_period_ms));
            continue;
        }

        if (f.magic != 0xCAFE || f.version != 1)
        {
            ESP_LOGW(TAG, "Bad header: magic=0x%04X ver=%u seq=%" PRIu32,
                     f.magic, f.version, f.seq);
            vTaskDelay(pdMS_TO_TICKS(g_telemetry_period_ms));
            continue;
        }

        if (!telemetry_crc_ok(&f))
        {
            ESP_LOGW(TAG, "CRC mismatch (seq=%" PRIu32 ")", f.seq);
            vTaskDelay(pdMS_TO_TICKS(g_telemetry_period_ms));
            continue;
        }

        // Print telemetry in a format easy to parse with Python
        printf("TELEM,%" PRIu32 ",%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%u,%u,%u\n",
               f.seq,
               f.x[0], f.x[1], f.x[2], f.x[3], f.x[4], f.x[5],
               (unsigned)f.duty[0], (unsigned)f.duty[1], (unsigned)f.duty[2]);

        vTaskDelay(pdMS_TO_TICKS(g_telemetry_period_ms));
    }
}

void app_main(void)
{
    spi_master_init();
    command_finalize_crc(&g_cmd);

    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(console_uart_task, "console", 4096, NULL, 4, NULL, 1);
}