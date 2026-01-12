
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "driver/spi_slave.h"

static const char *TAG_PWM = "MCPWM";
static const char *TAG_ADC = "ADC";
static const char *TAG_SPI = "CTRL_SPI_SLAVE";

// MCPWM frequency, initial duty and GPIO mapping
#define PWM_FREQUENCY_HZ 20000 // 20 kHz
#define PWM_INIT_DUTY_PC 0.0f  // Duty Cycle em porcentagem (0 a 100)
#define U_L_GPIO (22)          // Pino do PWM do ramo inferior da fase U
#define V_L_GPIO (21)          // Pino do PWM do ramo inferior da fase V
#define W_L_GPIO (5)           // Pino do PWM do ramo inferior da fase W
#define U_U_GPIO (23)          // Pino do PWM do ramo superior da fase U
#define V_U_GPIO (3)           // Pino do PWM do ramo superior da fase V
#define W_U_GPIO (18)          // Pino do PWM do ramo superior da fase W
#define U_ERR_GPIO (25)        // Pino de erro da fase U
#define V_ERR_GPIO (19)        // Pino de erro da fase V
#define W_ERR_GPIO (17)        // Pino de erro da fase W

// CONTROL PERIOD
#define CONTROL_RATE_HZ 1000
#define CONTROL_PERIOD_TICKS pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ)

// ADC
#define ADC_TASK_RATE_HZ 1000
#define ADC_PERIOD_TICKS pdMS_TO_TICKS(1000 / ADC_TASK_RATE_HZ) // 1 tick at 1kHz
#define ADC_SAMPLES_PER_CH 2
#define ADC_WIDTH_CFG ADC_WIDTH_BIT_12
#define ADC_ATTEN_CFG ADC_ATTEN_DB_11
#define GANHO_TENSAO_ENTRADA 23.4 / 0.804 //------- numero de leituras do ADC
#define GANHO_TENSAO_SAIDA 23.0 / 0.576   //------- numero de leituras do ADC
#define GANHO_TENSAO_CORRENTE 2.2         //----------------- numero de leituras do ADC

// SPI
#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK 14
#define PIN_CS 27
#define SPI_HOST HSPI_HOST

// TASK CORES
#define CORE_ADC 0
#define CORE_CONTROL 1
#define CORE_SPI 1

// TASK PRIORITIES
#define PRIO_CONTROL 20 // HIGH priority
#define PRIO_ADC 10
#define PRIO_SPI 5

// Mode selection
typedef enum
{
    PWM_MODE_BOOST = 0,
    PWM_MODE_BUCK = 1,
} pwm_mode_t;

// Packing struct
typedef struct
{
    mcpwm_unit_t unit;
    pwm_mode_t mode;
} mcpwm_task_params_t;

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
    "V_in",
    "V_out",
    "IL1",
    "IL2",
    "IL3",
    "IL_out",
};

static const adc1_channel_t s_adc1_sig_channel[ADC_SIG_COUNT] = {
    [ADC_SIG_V_IN] = ADC1_CHANNEL_0,   // GPIO36
    [ADC_SIG_V_OUT] = ADC1_CHANNEL_3,  // GPIO39
    [ADC_SIG_IL1] = ADC1_CHANNEL_6,    // GPIO34
    [ADC_SIG_IL2] = ADC1_CHANNEL_7,    // GPIO35
    [ADC_SIG_IL3] = ADC1_CHANNEL_4,    // GPIO32
    [ADC_SIG_IL_OUT] = ADC1_CHANNEL_5, // GPIO33
};

typedef struct
{
    int avg_raw[ADC_SIG_COUNT];
} adc1_avg_result_t;

static adc1_avg_result_t s_adc_res;

// ===== Telemetry mailbox for SPI (6 floats + 3 duty) =====
// You can map these to your real states/control outputs.
#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;  // 0xCAFE
    uint8_t version; // 1
    uint8_t reserved0;
    uint32_t seq; // increments each control tick

    float x[6];       // states
    uint16_t duty[3]; // duty scaled (0..1000, for example)
    uint16_t reserved1;

    uint32_t crc32; // CRC over all prior bytes
} telemetry_frame_t;
#pragma pack(pop)

static telemetry_frame_t telem_a, telem_b;
static telemetry_frame_t *volatile telem_current = &telem_a;
static portMUX_TYPE telem_mux = portMUX_INITIALIZER_UNLOCKED;

// Track latest duty for telemetry
static volatile float g_last_duty_u = 0.0f;
static volatile float g_last_duty_v = 0.0f;
static volatile float g_last_duty_w = 0.0f;

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
    uint32_t adc_reading = 0;
    for (int i = 0; i < samples; i++)
    {
        adc_reading += adc1_get_raw(ch);
    }
    adc_reading /= samples;
    return adc_reading;
}

float convertToVoltage(float adc_value)
{
    if (adc_value > 2890)
    {
        return 0.0014 * adc_value - 0.4103;
    }
    else
    {
        return 0.0008207742 * adc_value + 0.1391236;
    }
}

static void adc_sample_task(void *arg)
{
    (void)arg;
    adc1_init_legacy();

    TickType_t last = xTaskGetTickCount();

    while (1)
    {
        // Read & average all signals
        for (int i = 0; i < ADC_SIG_COUNT; i++)
        {
            s_adc_res.avg_raw[i] = adc1_read_avg(s_adc1_sig_channel[i], ADC_SAMPLES_PER_CH);
        }

        // Log slowly to avoid flooding (every 100ms)
        static int decim = 0;
        if (++decim >= 100)
        {
            decim = 0;
            ESP_LOGI(TAG_ADC,
                     "%s=%d %s=%d %s=%d %s=%d %s=%d %s=%d",
                     s_adc_sig_name[ADC_SIG_V_IN], s_adc_res.avg_raw[ADC_SIG_V_IN],
                     s_adc_sig_name[ADC_SIG_V_OUT], s_adc_res.avg_raw[ADC_SIG_V_OUT],
                     s_adc_sig_name[ADC_SIG_IL1], s_adc_res.avg_raw[ADC_SIG_IL1],
                     s_adc_sig_name[ADC_SIG_IL2], s_adc_res.avg_raw[ADC_SIG_IL2],
                     s_adc_sig_name[ADC_SIG_IL3], s_adc_res.avg_raw[ADC_SIG_IL3],
                     s_adc_sig_name[ADC_SIG_IL_OUT], s_adc_res.avg_raw[ADC_SIG_IL_OUT]);
        }

        vTaskDelayUntil(&last, ADC_PERIOD_TICKS);
    }
}

// Helper to clamp
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi
                                    : x;
}

// Configure MCPWM timer, operators, GPIOs, and faults
static esp_err_t mcpwm_setup(mcpwm_unit_t unit)
{
    // 1. Configure GPIOs for PWM outputs (A and B for each timer)
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM0A, U_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM0B, U_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM1A, V_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM1B, V_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM2A, W_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM2B, W_U_GPIO));

    // 2. Configure fault GPIOs with internal pull-down resistors
    gpio_config_t fault_gpio_conf = {
        .pin_bit_mask = (1ULL << U_ERR_GPIO) | (1ULL << V_ERR_GPIO) | (1ULL << W_ERR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&fault_gpio_conf));

    // 3. Configure GPIOs for fault inputs
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM_FAULT_0, U_ERR_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM_FAULT_1, V_ERR_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM_FAULT_2, W_ERR_GPIO));

    // 4. Timer configuration (same for all three timers)
    mcpwm_config_t pwm_cfg = {
        .frequency = PWM_FREQUENCY_HZ,
        .cmpr_a = PWM_INIT_DUTY_PC,
        .cmpr_b = PWM_INIT_DUTY_PC,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_ERROR_CHECK(mcpwm_init(unit, MCPWM_TIMER_0, &pwm_cfg));
    ESP_ERROR_CHECK(mcpwm_init(unit, MCPWM_TIMER_1, &pwm_cfg));
    ESP_ERROR_CHECK(mcpwm_init(unit, MCPWM_TIMER_2, &pwm_cfg));

    // 5. Enable Synchronization between timers - phase_val between 0-999
    mcpwm_sync_config_t sync_cfg = {
        .sync_sig = MCPWM_SELECT_TIMER0_SYNC,
        .timer_val = 0,
        .count_direction = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_sync_configure(unit, MCPWM_TIMER_0, &sync_cfg));
    sync_cfg.timer_val = 333;
    ESP_ERROR_CHECK(mcpwm_sync_configure(unit, MCPWM_TIMER_1, &sync_cfg));
    sync_cfg.timer_val = 667;
    ESP_ERROR_CHECK(mcpwm_sync_configure(unit, MCPWM_TIMER_2, &sync_cfg));

    // 6. Initialize fault detection (IDF 4.4.x: use HIGH level trigger)
    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F0));
    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F1));
    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F2));

    // 7. Configure fault actions (cycle-by-cycle mode)
    mcpwm_output_action_t action_a = MCPWM_ACTION_FORCE_LOW;
    mcpwm_output_action_t action_b = MCPWM_ACTION_FORCE_LOW;

    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_0, MCPWM_SELECT_F0, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_1, MCPWM_SELECT_F1, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_2, MCPWM_SELECT_F2, action_a, action_b));

    return ESP_OK;
}

// Update duty cycle for all three phases based on mode
static pwm_mode_t g_current_mode = PWM_MODE_BOOST;
void update_duty(mcpwm_unit_t unit, float duty_u, float duty_v, float duty_w, pwm_mode_t mode)
{
    duty_u = clampf(duty_u, 0.0f, 100.0f);
    duty_v = clampf(duty_v, 0.0f, 100.0f);
    duty_w = clampf(duty_w, 0.0f, 100.0f);

    static pwm_mode_t last_mode = PWM_MODE_BOOST;
    if (mode != last_mode)
    {
        ESP_LOGI(TAG_PWM, "Mode changed to: %s", mode == PWM_MODE_BOOST ? "BOOST" : "BUCK");
        last_mode = mode;
        g_current_mode = mode;
        if (mode == PWM_MODE_BOOST)
        {
            mcpwm_set_signal_low(unit, MCPWM_TIMER_0, MCPWM_OPR_B);
            mcpwm_set_signal_low(unit, MCPWM_TIMER_1, MCPWM_OPR_B);
            mcpwm_set_signal_low(unit, MCPWM_TIMER_2, MCPWM_OPR_B);
        }
        else
        {
            mcpwm_set_signal_low(unit, MCPWM_TIMER_0, MCPWM_OPR_A);
            mcpwm_set_signal_low(unit, MCPWM_TIMER_1, MCPWM_OPR_A);
            mcpwm_set_signal_low(unit, MCPWM_TIMER_2, MCPWM_OPR_A);
        }
    }

    if (mode == PWM_MODE_BOOST)
    {
        mcpwm_set_duty(unit, MCPWM_TIMER_0, MCPWM_OPR_A, duty_u);
        mcpwm_set_duty(unit, MCPWM_TIMER_1, MCPWM_OPR_A, duty_v);
        mcpwm_set_duty(unit, MCPWM_TIMER_2, MCPWM_OPR_A, duty_w);
    }
    else
    {
        mcpwm_set_duty(unit, MCPWM_TIMER_0, MCPWM_OPR_B, duty_u);
        mcpwm_set_duty(unit, MCPWM_TIMER_1, MCPWM_OPR_B, duty_v);
        mcpwm_set_duty(unit, MCPWM_TIMER_2, MCPWM_OPR_B, duty_w);
    }
}

// ===== SPI SLAVE =====
static void spi_slave_init_simple(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 2,
        .flags = 0,
    };

    // Optional pull-ups
    gpio_set_pull_mode(PIN_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_SCK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_CS, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));
}

static void spi_telemetry_task(void *arg)
{
    (void)arg;

    static WORD_ALIGNED_ATTR telemetry_frame_t tx_frame;
    static WORD_ALIGNED_ATTR uint8_t rx_dummy[sizeof(telemetry_frame_t)];

    while (1)
    {
        // Snapshot the latest telemetry pointer under a very short critical section
        taskENTER_CRITICAL(&telem_mux);
        telemetry_frame_t snap = *telem_current;
        taskEXIT_CRITICAL(&telem_mux);

        memcpy(&tx_frame, &snap, sizeof(tx_frame));
        memset(rx_dummy, 0, sizeof(rx_dummy));

        spi_slave_transaction_t t = {
            .length = 8 * sizeof(tx_frame),
            .tx_buffer = &tx_frame,
            .rx_buffer = rx_dummy,
        };

        esp_err_t err = spi_slave_transmit(SPI_HOST, &t, portMAX_DELAY);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_SPI, "spi_slave_transmit failed: %s", esp_err_to_name(err));
        }
    }
}

// Periodic control task
void mcpwm_control_task(void *arg)
{
    mcpwm_task_params_t *params = (mcpwm_task_params_t *)arg;
    mcpwm_unit_t unit = params->unit;
    pwm_mode_t mode = params->mode;

    float duty = 0.0f;
    float step = 0.5f;

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);

    for (;;)
    {
        update_duty(unit, duty, duty, duty, mode);

        duty += step;
        if (duty >= 100.0f)
        {
            duty = 100.0f;
            step = -step;
        }
        if (duty <= 0.0f)
        {
            duty = 0.0f;
            step = -step;
        }

        vTaskDelayUntil(&last_wake_time, period);
    }
}

void app_main(void)
{
    mcpwm_unit_t unit = MCPWM_UNIT_0;
    ESP_ERROR_CHECK(mcpwm_setup(unit));
    ESP_LOGI(TAG_PWM, "MCPWM setup complete");

    static mcpwm_task_params_t ctrl_params = {
        .unit = MCPWM_UNIT_0,
        .mode = PWM_MODE_BOOST,
    };

    // SPI slave init + task (low priority)
    spi_slave_init_simple();
    xTaskCreatePinnedToCore(
        spi_telemetry_task,
        "spi_telemetry",
        4096,
        NULL,
        PRIO_SPI,
        NULL,
        CORE_SPI);

    xTaskCreatePinnedToCore(
        mcpwm_control_task,
        "mcpwm_ctrl",
        4096,
        &ctrl_params,
        PRIO_CONTROL,
        NULL,
        CORE_CONTROL);

    xTaskCreatePinnedToCore(
        adc_sample_task,
        "adc",
        4096,
        NULL,
        PRIO_ADC,
        NULL,
        CORE_ADC);
}