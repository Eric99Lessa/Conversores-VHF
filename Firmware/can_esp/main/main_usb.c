#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_crc.h"

#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK 14
#define PIN_CS 27
#define SPI_HOST VSPI_HOST

static const char *TAG = "SPI_MASTER";

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
    uint16_t reserved1;

    uint32_t crc32;
} telemetry_frame_t;
#pragma pack(pop)

_Static_assert(sizeof(telemetry_frame_t) == (2 + 1 + 1 + 4 + 6 * 4 + 3 * 2 + 2 + 4),
               "Unexpected telemetry frame size");

// ===== COMMAND FRAME (master -> slave) =====
typedef enum
{
    CTRL_MODE_OPEN_LOOP = 0,
    CTRL_MODE_CLOSED_LOOP = 1,
} control_mode_t;

typedef enum
{
    SUBMODE_FIXED = 0,
    SUBMODE_RAMP = 1,
    SUBMODE_PI = 2,
    SUBMODE_IDA_PBC = 3,
} control_submode_t;

#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;  // 0xBEEF
    uint8_t version; // 1
    uint8_t mode;    // control_mode_t
    uint8_t submode; // control_submode_t
    uint8_t reserved0;
    uint16_t duty_cmd[3]; // Permille (0..1000)
    uint32_t crc32;
} command_frame_t;
#pragma pack(pop)

_Static_assert(sizeof(command_frame_t) == (2 + 1 + 1 + 1 + 1 + 3 * 2 + 4),
               "Unexpected command frame size");

static spi_device_handle_t spi;

static command_frame_t g_cmd = {
    .magic = 0xBEEF,
    .version = 1,
    .mode = CTRL_MODE_OPEN_LOOP,
    .submode = SUBMODE_FIXED,
    .reserved0 = 0,
    .duty_cmd = {200, 200, 200}, // default 20.0%
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
        .max_transfer_sz = 128,
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

    ESP_LOGI(TAG, "SPI master initialized (mode0, 2MHz). Telem=%u bytes Cmd=%u bytes",
             (unsigned)sizeof(telemetry_frame_t), (unsigned)sizeof(command_frame_t));
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

// Full-duplex: send command, receive telemetry
static esp_err_t spi_exchange(telemetry_frame_t *telem_in)
{
    command_frame_t cmd;
    taskENTER_CRITICAL(&cmd_mux);
    cmd = g_cmd;
    taskEXIT_CRITICAL(&cmd_mux);

    uint8_t tx_buf[sizeof(telemetry_frame_t)];
    memset(tx_buf, 0, sizeof(tx_buf));
    memcpy(tx_buf, &cmd, sizeof(command_frame_t));

    memset(telem_in, 0, sizeof(*telem_in));

    spi_transaction_t t = {
        .length = 8 * sizeof(telemetry_frame_t),
        .tx_buffer = tx_buf,
        .rx_buffer = telem_in,
    };

    return spi_device_transmit(spi, &t);
}

// ===== Console parsing =====
static void console_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Console ready. Commands (all values in permille 0..1000):");
    ESP_LOGI(TAG, "  Mode Open <U> <V> <W>           e.g. Mode Open 500 600 250");
    ESP_LOGI(TAG, "  Mode Open Ramp <step> <base>    e.g. Mode Open Ramp 5 200");
    ESP_LOGI(TAG, "  Mode Closed PI");
    ESP_LOGI(TAG, "  Mode Closed IDA-PBC");

    char line[128];

    while (1)
    {
        if (fgets(line, sizeof(line), stdin) == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        char *argv[8];
        int argc = 0;
        char *p = strtok(line, " \t\n\r");
        while (p && argc < 8)
        {
            argv[argc++] = p;
            p = strtok(NULL, " \t\n\r");
        }

        if (argc < 2 || strcasecmp(argv[0], "Mode") != 0)
        {
            continue;
        }

        taskENTER_CRITICAL(&cmd_mux);

        if (strcasecmp(argv[1], "Open") == 0)
        {
            g_cmd.mode = CTRL_MODE_OPEN_LOOP;

            if (argc >= 5 && strcasecmp(argv[2], "Ramp") != 0)
            {
                // Mode Open <U> <V> <W>
                g_cmd.submode = SUBMODE_FIXED;
                g_cmd.duty_cmd[0] = (uint16_t)atoi(argv[2]);
                g_cmd.duty_cmd[1] = (uint16_t)atoi(argv[3]);
                g_cmd.duty_cmd[2] = (uint16_t)atoi(argv[4]);
                ESP_LOGI(TAG, "Set OPEN (Fixed): U=%u V=%u W=%u permille",
                         g_cmd.duty_cmd[0], g_cmd.duty_cmd[1], g_cmd.duty_cmd[2]);
            }
            else if (argc >= 5 && strcasecmp(argv[2], "Ramp") == 0)
            {
                // Mode Open Ramp <step> <base>
                g_cmd.submode = SUBMODE_RAMP;
                g_cmd.duty_cmd[0] = (uint16_t)atoi(argv[3]); // step
                g_cmd.duty_cmd[1] = (uint16_t)atoi(argv[4]); // base
                g_cmd.duty_cmd[2] = 0;
                ESP_LOGI(TAG, "Set OPEN (Ramp): step=%u base=%u permille",
                         g_cmd.duty_cmd[0], g_cmd.duty_cmd[1]);
            }
        }
        else if (strcasecmp(argv[1], "Closed") == 0)
        {
            g_cmd.mode = CTRL_MODE_CLOSED_LOOP;

            if (argc >= 3 && strcasecmp(argv[2], "PI") == 0)
            {
                g_cmd.submode = SUBMODE_PI;
                ESP_LOGI(TAG, "Set CLOSED (PI)");
            }
            else if (argc >= 3 && strcasecmp(argv[2], "IDA-PBC") == 0)
            {
                g_cmd.submode = SUBMODE_IDA_PBC;
                ESP_LOGI(TAG, "Set CLOSED (IDA-PBC)");
            }
        }

        command_finalize_crc(&g_cmd);
        taskEXIT_CRITICAL(&cmd_mux);
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
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (f.magic != 0xCAFE || f.version != 1)
        {
            ESP_LOGW(TAG, "Bad header: magic=0x%04X ver=%u seq=%" PRIu32,
                     f.magic, f.version, f.seq);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!telemetry_crc_ok(&f))
        {
            ESP_LOGW(TAG, "CRC mismatch (seq=%" PRIu32 ")", f.seq);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // Log the received telemetry
        ESP_LOGI(TAG, "seq=%" PRIu32, f.seq);

        ESP_LOGI(TAG, "x: [%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f]",
                 f.x[0], f.x[1], f.x[2], f.x[3], f.x[4], f.x[5]);

        ESP_LOGI(TAG, "duty (permille): U=%u V=%u W=%u  (=> %%: %0.1f %0.1f %0.1f)",
                 (unsigned)f.duty[0], (unsigned)f.duty[1], (unsigned)f.duty[2],
                 f.duty[0] / 10.0f, f.duty[1] / 10.0f, f.duty[2] / 10.0f);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    spi_master_init();

    // Initialize default command with CRC
    command_finalize_crc(&g_cmd);

    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 4, NULL, 1);
}