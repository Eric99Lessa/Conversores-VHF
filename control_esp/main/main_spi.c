#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_crc.h"
#include "esp_heap_caps.h"

static const char *TAG_SPI = "SPI_SLAVE_TEST";

#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK 14
#define PIN_CS 27
#define SPI_HOST HSPI_HOST

#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;  // 0xCAFE
    uint8_t version; // 1
    uint8_t rsv0;
    uint32_t seq;
    uint32_t crc32;
} test_frame_t;
#pragma pack(pop)

static void spi_slave_init_simple(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 2,
        .flags = 0,
    };

    gpio_set_pull_mode(PIN_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_SCK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_CS, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
}

static void spi_task(void *arg)
{
    (void)arg;

    test_frame_t *tx = heap_caps_malloc(sizeof(test_frame_t), MALLOC_CAP_DMA);
    uint8_t *rx = heap_caps_malloc(sizeof(test_frame_t), MALLOC_CAP_DMA);

    if (!tx || !rx)
    {
        ESP_LOGE(TAG_SPI, "DMA alloc failed");
        vTaskDelete(NULL);
    }

    uint32_t seq = 0;

    while (1)
    {
        seq++;

        memset(tx, 0, sizeof(*tx));
        tx->magic = 0xCAFE;
        tx->version = 1;
        tx->seq = seq;
        tx->crc32 = 0;
        tx->crc32 = esp_crc32_le(0, (const uint8_t *)tx, sizeof(*tx) - sizeof(tx->crc32));

        memset(rx, 0, sizeof(test_frame_t));

        spi_slave_transaction_t t = {
            .length = 8 * sizeof(test_frame_t),
            .tx_buffer = tx,
            .rx_buffer = rx,
        };

        esp_err_t err = spi_slave_transmit(SPI_HOST, &t, portMAX_DELAY);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_SPI, "spi_slave_transmit failed: %s", esp_err_to_name(err));
        }
    }
}

void app_main(void)
{
    spi_slave_init_simple();
    ESP_LOGI(TAG_SPI, "SPI slave test ready (mode0). Waiting for master...");
    xTaskCreate(spi_task, "spi", 4096, NULL, 5, NULL);
}