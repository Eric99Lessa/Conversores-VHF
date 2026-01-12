#include <string.h>
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

static const char *TAG = "SPI_MASTER_TEST";

#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;  // 0xCAFE
    uint8_t version; // 1
    uint8_t rsv0;
    uint32_t seq;
    uint32_t crc32; // CRC32 (LE) over all prior bytes
} test_frame_t;
#pragma pack(pop)

_Static_assert(sizeof(test_frame_t) == (2 + 1 + 1 + 4 + 4), "Unexpected test_frame_t size");

static spi_device_handle_t spi;

static void spi_master_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500 * 1000, // start at 500kHz; you can increase later
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .flags = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));

    ESP_LOGI(TAG, "SPI master init OK (mode0, 500kHz). Frame=%u bytes", (unsigned)sizeof(test_frame_t));
}

static bool test_crc_ok(const test_frame_t *f)
{
    test_frame_t tmp;
    memcpy(&tmp, f, sizeof(tmp));
    uint32_t saved = tmp.crc32;
    tmp.crc32 = 0;

    uint32_t crc = esp_crc32_le(0, (const uint8_t *)&tmp, sizeof(tmp) - sizeof(tmp.crc32));
    return (crc == saved);
}

static esp_err_t spi_read_test_frame(test_frame_t *out)
{
    uint8_t tx_dummy[sizeof(test_frame_t)];
    memset(tx_dummy, 0, sizeof(tx_dummy));
    memset(out, 0, sizeof(*out));

    spi_transaction_t t = {
        .length = 8 * sizeof(test_frame_t),
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
        test_frame_t f;
        esp_err_t err = spi_read_test_frame(&f);
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
            ESP_LOGI(TAG, "Raw: %02X %02X %02X %02X %02X %02X %02X %02X",
                     ((uint8_t *)&f)[0], ((uint8_t *)&f)[1], ((uint8_t *)&f)[2], ((uint8_t *)&f)[3],
                     ((uint8_t *)&f)[4], ((uint8_t *)&f)[5], ((uint8_t *)&f)[6], ((uint8_t *)&f)[7]);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!test_crc_ok(&f))
        {
            ESP_LOGW(TAG, "CRC mismatch (seq=%" PRIu32 "): recv=0x%08" PRIX32, f.seq, f.crc32);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        ESP_LOGI(TAG, "OK: seq=%" PRIu32 " crc=0x%08" PRIX32, f.seq, f.crc32);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}