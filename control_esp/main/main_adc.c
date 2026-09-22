#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"

static const char *TAG_ADC = "ADC";

#define ADC_SAMPLES_PER_CH 2
#define ADC_WIDTH_CFG ADC_WIDTH_BIT_12
#define ADC_ATTEN_CFG ADC_ATTEN_DB_11

typedef enum
{
    ADC_SIG_V_IN = 0,
    ADC_SIG_V_OUT,
    ADC_SIG_IL1,
    ADC_SIG_IL2,
    ADC_SIG_IL3,
    ADC_SIG_IL_OUT,
    ADC_SIG_COUNT
} adc_signal_t;

static const char *s_adc_sig_name[ADC_SIG_COUNT] = {
    "V_in", "V_out", "IL1", "IL2", "IL3", "IL_out"};

static const adc1_channel_t s_adc1_sig_channel[ADC_SIG_COUNT] = {
    [ADC_SIG_V_IN] = ADC1_CHANNEL_0,   // GPIO36
    [ADC_SIG_V_OUT] = ADC1_CHANNEL_3,  // GPIO39
    [ADC_SIG_IL1] = ADC1_CHANNEL_6,    // GPIO34
    [ADC_SIG_IL2] = ADC1_CHANNEL_7,    // GPIO35
    [ADC_SIG_IL3] = ADC1_CHANNEL_4,    // GPIO32
    [ADC_SIG_IL_OUT] = ADC1_CHANNEL_5, // GPIO33
};

static void adc1_init_legacy(void)
{
    adc1_config_width(ADC_WIDTH_CFG);
    for (int i = 0; i < ADC_SIG_COUNT; i++)
    {
        adc1_config_channel_atten(s_adc1_sig_channel[i], ADC_ATTEN_CFG);
    }
    ESP_LOGI(TAG_ADC, "ADC1 configured (legacy). TickHz=%d", configTICK_RATE_HZ);
}

static inline int adc1_read_avg(adc1_channel_t ch, int samples)
{
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++)
    {
        sum += adc1_get_raw(ch);
    }
    return (int)(sum / samples);
}

static void adc_task(void *arg)
{
    (void)arg;
    adc1_init_legacy();

    while (1)
    {
        int v[ADC_SIG_COUNT];
        for (int i = 0; i < ADC_SIG_COUNT; i++)
        {
            v[i] = adc1_read_avg(s_adc1_sig_channel[i], ADC_SAMPLES_PER_CH);
        }

        ESP_LOGI(TAG_ADC,
                 "%s=%d %s=%d %s=%d %s=%d %s=%d %s=%d",
                 s_adc_sig_name[ADC_SIG_V_IN], v[ADC_SIG_V_IN],
                 s_adc_sig_name[ADC_SIG_V_OUT], v[ADC_SIG_V_OUT],
                 s_adc_sig_name[ADC_SIG_IL1], v[ADC_SIG_IL1],
                 s_adc_sig_name[ADC_SIG_IL2], v[ADC_SIG_IL2],
                 s_adc_sig_name[ADC_SIG_IL3], v[ADC_SIG_IL3],
                 s_adc_sig_name[ADC_SIG_IL_OUT], v[ADC_SIG_IL_OUT]);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    xTaskCreate(adc_task, "adc", 4096, NULL, 5, NULL);
}