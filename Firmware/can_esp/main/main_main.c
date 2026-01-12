#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_crc.h"

// ===== Pins (your wiring) =====
#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK 14
#define PIN_CS 27

#define SPI_HOST VSPI_HOST // SPI2 on ESP32

static const char *TAG = "SPI_MASTER";

// Must match the slave frame layout exactly
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

_Static_assert(sizeof(telemetry_frame_t) == (2 + 1 + 1 + 4 + 6 * 4 + 3 * 2 + 2 + 4), "Unexpected frame size");

static spi_device_handle_t spi;

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
        .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz (adjust as needed)
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .flags = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));

    ESP_LOGI(TAG, "SPI master initialized (mode0, 2MHz). Frame=%u bytes", (unsigned)sizeof(telemetry_frame_t));
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

static esp_err_t spi_read_telemetry(telemetry_frame_t *out)
{
    uint8_t tx_dummy[sizeof(telemetry_frame_t)];
    memset(tx_dummy, 0, sizeof(tx_dummy));
    memset(out, 0, sizeof(*out));

    spi_transaction_t t = {
        .length = 8 * sizeof(telemetry_frame_t),
        .tx_buffer = tx_dummy,
        .rx_buffer = out,
    };

    return spi_device_transmit(spi, &t);
}

void app_main(void)
{
    spi_master_init();

    while (1)
    {
        telemetry_frame_t f;
        esp_err_t err = spi_read_telemetry(&f);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(err));
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

        // Example: only ask occasionally (replace with "on CAN request")
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}