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
#include "esp_heap_caps.h"
#include "driver/uart.h"

static const char *TAG_PWM = "MCPWM";
static const char *TAG_ADC = "ADC";
static const char *TAG_SPI = "CTRL_SPI_SLAVE";

// MCPWM frequency and GPIO mapping
#define PWM_FREQUENCY_HZ 20000
#define PWM_INIT_DUTY_PC 0.0f
#define U_L_GPIO (22)
#define V_L_GPIO (21)
#define W_L_GPIO (5)
#define U_U_GPIO (23)
#define V_U_GPIO (3)
#define W_U_GPIO (18)
#define U_ERR_GPIO (1)
#define V_ERR_GPIO (19)
#define W_ERR_GPIO (17)

// CONTROL PERIOD
#define CONTROL_RATE_HZ 1000
#define CONTROL_PERIOD_TICKS pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ)

// ADC
#define ADC_TASK_RATE_HZ 1000
#define ADC_PERIOD_TICKS pdMS_TO_TICKS(1000 / ADC_TASK_RATE_HZ)
#define ADC_SAMPLES_PER_CH 4
#define ADC_WIDTH_CFG ADC_WIDTH_BIT_12
#define ADC_ATTEN_CFG ADC_ATTEN_DB_11
#define Vin_Gain 0.976087587027069f
#define Vout_Gain 0.890099355577886f
#define R1_In 9400.0f
#define R1_Out 14100.0f
#define Rm_In 100.0f
#define Rm_Out 100.0f
#define LF210_Gain 2000.0f // input/output
#define LV20P_Gain 0.4f    // input/output
#define R_IL 100.0f
#define R_Iout 100.0f
// SPI pins/host
#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK 14
#define PIN_CS 27
#define SPI_HOST VSPI_HOST

// Fixed SPI transfer size (MOSI and MISO) — MUST match master
#define SPI_XFER_BYTES 64

// TASK CORES / PRIORITIES
#define CORE_ADC 0
#define CORE_CONTROL 1
#define CORE_SPI 1
#define PRIO_CONTROL 20
#define PRIO_ADC 10
#define PRIO_SPI 5

// PWM mode selection
typedef enum
{
    PWM_MODE_BOOST = 0,
    PWM_MODE_BUCK = 1,
} pwm_mode_t;

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

static const adc1_channel_t s_adc1_sig_channel[ADC_SIG_COUNT] = {
    [ADC_SIG_V_IN] = ADC1_CHANNEL_0,
    [ADC_SIG_V_OUT] = ADC1_CHANNEL_3,
    [ADC_SIG_IL1] = ADC1_CHANNEL_7,
    [ADC_SIG_IL2] = ADC1_CHANNEL_4,
    [ADC_SIG_IL3] = ADC1_CHANNEL_5,
    [ADC_SIG_IL_OUT] = ADC1_CHANNEL_6,
};

typedef struct
{
    int avg_raw[ADC_SIG_COUNT];
} adc1_avg_result_t;

static adc1_avg_result_t s_adc_res;

// ===== TELEMETRY FRAME (slave → master) =====
#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;  // 0xCAFE
    uint8_t version; // 1
    uint8_t reserved0;
    uint32_t seq;

    float x[6];
    uint16_t duty[3]; // permille 0..1000

    uint32_t crc32;
} telemetry_frame_t;
#pragma pack(pop)

_Static_assert(sizeof(telemetry_frame_t) == (2 + 1 + 1 + 4 + 6 * 4 + 3 * 2 + 4),
               "Unexpected telemetry frame size");

// ===== COMMAND FRAME (master → slave) =====
typedef enum
{
    CTRL_MODE_DUTY = 0,
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

    uint16_t duty_cmd[3]; // permille 0..1000 (used when mode == CTRL_MODE_DUTY)

    uint32_t crc32;
} command_frame_t;
#pragma pack(pop)

_Static_assert(sizeof(command_frame_t) == (2 + 1 + 1 + 1 + 3 * 2 + 4),
               "Unexpected command frame size");

_Static_assert(SPI_XFER_BYTES >= sizeof(telemetry_frame_t), "SPI_XFER_BYTES too small for telemetry");
_Static_assert(SPI_XFER_BYTES >= sizeof(command_frame_t), "SPI_XFER_BYTES too small for command");

// ===== SHARED STATE =====
typedef struct
{
    float x[6];
    uint16_t duty[3]; // permille
} telemetry_state_t;

static telemetry_state_t g_telem_state;
static portMUX_TYPE telem_mux = portMUX_INITIALIZER_UNLOCKED;

// Last valid command from master
static command_frame_t g_cmd = {
    .magic = 0xBEEF,
    .version = 1,
    .mode = CTRL_MODE_DUTY,
    .reserved0 = 0,
    .duty_cmd = {200, 200, 200},
    .crc32 = 0,
};
static portMUX_TYPE cmd_mux = portMUX_INITIALIZER_UNLOCKED;

// ===== CRC HELPERS =====
static bool command_crc_ok(const command_frame_t *c)
{
    uint32_t saved = c->crc32;
    command_frame_t tmp;
    memcpy(&tmp, c, sizeof(tmp));
    tmp.crc32 = 0;
    uint32_t crc = esp_crc32_le(0, (const uint8_t *)&tmp, sizeof(tmp) - sizeof(tmp.crc32));
    return (crc == saved);
}

// ===== ADC =====
static void adc1_init_legacy(void)
{
    adc1_config_width(ADC_WIDTH_CFG);
    for (int i = 0; i < ADC_SIG_COUNT; i++)
    {
        adc1_config_channel_atten(s_adc1_sig_channel[i], ADC_ATTEN_CFG);
    }
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

float digitalToAnalog(float adc_value)
{
    if (adc_value == 0)
    {
        return 0.0f;
    }
    if (adc_value > 2890)
    {
        return 0.0014f * adc_value - 0.4103f;
    }
    else
    {
        return 0.0008207742f * adc_value + 0.1391236f;
    }
}

static void adc_sample_task(void *arg)
{
    (void)arg;
    adc1_init_legacy();

    TickType_t last = xTaskGetTickCount();

    while (1)
    {
        for (int i = 0; i < ADC_SIG_COUNT - 1; i++)
        {
            s_adc_res.avg_raw[i] = adc1_read_avg(s_adc1_sig_channel[i], ADC_SAMPLES_PER_CH);
        }

        taskENTER_CRITICAL(&telem_mux);
        // g_telem_state.x[0] = Vin_Gain * R1_In * LV20P_Gain * digitalToAnalog((float)s_adc_res.avg_raw[ADC_SIG_V_IN]) / Rm_In;
        // g_telem_state.x[1] = Vout_Gain * R1_Out * LV20P_Gain * digitalToAnalog((float)s_adc_res.avg_raw[ADC_SIG_V_OUT]) / Rm_Out;
        g_telem_state.x[2] = LF210_Gain * digitalToAnalog((float)s_adc_res.avg_raw[ADC_SIG_IL1]) / R_IL;
        g_telem_state.x[3] = LF210_Gain * digitalToAnalog((float)s_adc_res.avg_raw[ADC_SIG_IL2]) / R_IL;
        g_telem_state.x[4] = LF210_Gain * digitalToAnalog((float)s_adc_res.avg_raw[ADC_SIG_IL3]) / R_IL;
        // g_telem_state.x[5] = LF210_Gain * digitalToAnalog((float)s_adc_res.avg_raw[ADC_SIG_IL_OUT]) / R_Iout;
        // g_telem_state.x[0] = (float)s_adc_res.avg_raw[ADC_SIG_V_IN];
        // g_telem_state.x[1] = (float)s_adc_res.avg_raw[ADC_SIG_V_OUT];
        // g_telem_state.x[2] = (float)s_adc_res.avg_raw[ADC_SIG_IL1];
        // g_telem_state.x[3] = (float)s_adc_res.avg_raw[ADC_SIG_IL2];
        // g_telem_state.x[4] = (float)s_adc_res.avg_raw[ADC_SIG_IL3];
        g_telem_state.x[0] = (float)s_adc_res.avg_raw[ADC_SIG_V_IN] * 0.029724924303682 + 3.361912841627126;
        g_telem_state.x[1] = (float)s_adc_res.avg_raw[ADC_SIG_V_OUT] * 0.047925930496814 - 0.566484076397789;
        g_telem_state.x[5] = 0.0f;
        taskEXIT_CRITICAL(&telem_mux);

        vTaskDelayUntil(&last, ADC_PERIOD_TICKS);
    }
}

// ===== MCPWM =====
static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi
                                    : x;
}

static inline uint16_t clamp_u16(int v, int lo, int hi)
{
    if (v < lo)
        v = lo;
    if (v > hi)
        v = hi;
    return (uint16_t)v;
}

static esp_err_t mcpwm_setup(mcpwm_unit_t unit)
{
    // GPIO INIT PWM
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM0A, U_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM0B, U_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM1A, V_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM1B, V_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM2A, W_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM2B, W_U_GPIO));

    // GPIO INIT FAULT
    gpio_config_t fault_gpio_conf = {
        .pin_bit_mask = (1ULL << U_ERR_GPIO) | (1ULL << V_ERR_GPIO) | (1ULL << W_ERR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&fault_gpio_conf));

    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM_FAULT_0, U_ERR_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM_FAULT_1, V_ERR_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM_FAULT_2, W_ERR_GPIO));

    // Frequency and initial value
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

    mcpwm_set_timer_sync_output(unit, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);

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

    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F0));
    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F1));
    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F2));

    mcpwm_output_action_t action_a = MCPWM_ACTION_FORCE_LOW;
    mcpwm_output_action_t action_b = MCPWM_ACTION_FORCE_LOW;

    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_0, MCPWM_SELECT_F0, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_1, MCPWM_SELECT_F1, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_2, MCPWM_SELECT_F2, action_a, action_b));

    return ESP_OK;
}

void update_duty(mcpwm_unit_t unit, float duty_u, float duty_v, float duty_w, pwm_mode_t mode)
{
    duty_u = clampf(duty_u, 0.0f, 100.0f);
    duty_v = clampf(duty_v, 0.0f, 100.0f);
    duty_w = clampf(duty_w, 0.0f, 100.0f);

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

// ===== CONTROL MODE FUNCTIONS (fill in later) =====
// All duty outputs are permille 0..1000.

static void control_compute_duty_mode_duty(const telemetry_state_t *state,
                                           const command_frame_t *cmd,
                                           uint16_t duty_out[3])
{
    (void)state;
    duty_out[0] = clamp_u16(cmd->duty_cmd[0], 0, 1000);
    duty_out[1] = clamp_u16(cmd->duty_cmd[1], 0, 1000);
    duty_out[2] = clamp_u16(cmd->duty_cmd[2], 0, 1000);
}

static void control_compute_duty_mode_pi(const telemetry_state_t *state,
                                         const command_frame_t *cmd,
                                         uint16_t duty_out[3])
{
    (void)state;
    (void)cmd;

    // TODO: implement PI control law here
    // placeholder:
    duty_out[0] = 300;
    duty_out[1] = 300;
    duty_out[2] = 300;
}

static void control_compute_duty_mode_idapbc(const telemetry_state_t *state,
                                             const command_frame_t *cmd,
                                             uint16_t duty_out[3])
{
    (void)state;
    (void)cmd;

    // TODO: implement IDA-PBC control law here
    // placeholder:
    duty_out[0] = 400;
    duty_out[1] = 400;
    duty_out[2] = 400;
}

// ===== CONTROL TASK (1 kHz) =====
void mcpwm_control_task(void *arg)
{
    mcpwm_task_params_t *params = (mcpwm_task_params_t *)arg;
    mcpwm_unit_t unit = params->unit;

    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        // Snapshot latest command
        command_frame_t cmd;
        taskENTER_CRITICAL(&cmd_mux);
        cmd = g_cmd;
        taskEXIT_CRITICAL(&cmd_mux);

        // Snapshot latest measured state (for PI/IDA-PBC use)
        telemetry_state_t state;
        taskENTER_CRITICAL(&telem_mux);
        state = g_telem_state;
        taskEXIT_CRITICAL(&telem_mux);

        // Compute duty based on selected mode
        uint16_t duty_perm[3] = {0, 0, 0};

        switch ((control_mode_t)cmd.mode)
        {
        case CTRL_MODE_DUTY:
            control_compute_duty_mode_duty(&state, &cmd, duty_perm);
            break;

        case CTRL_MODE_PI:
            control_compute_duty_mode_pi(&state, &cmd, duty_perm);
            break;

        case CTRL_MODE_IDA_PBC:
            control_compute_duty_mode_idapbc(&state, &cmd, duty_perm);
            break;

        default:
            // Unknown mode -> safe output
            duty_perm[0] = 0;
            duty_perm[1] = 0;
            duty_perm[2] = 0;
            break;
        }

        // Apply duty (permille -> percent)
        update_duty(unit,
                    duty_perm[0] / 10.0f,
                    duty_perm[1] / 10.0f,
                    duty_perm[2] / 10.0f,
                    params->mode);

        // Publish applied duty to telemetry
        taskENTER_CRITICAL(&telem_mux);
        g_telem_state.duty[0] = duty_perm[0];
        g_telem_state.duty[1] = duty_perm[1];
        g_telem_state.duty[2] = duty_perm[2];
        taskEXIT_CRITICAL(&telem_mux);

        vTaskDelayUntil(&last_wake_time, CONTROL_PERIOD_TICKS);
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
        .max_transfer_sz = SPI_XFER_BYTES,
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

    ESP_LOGI(TAG_SPI, "SPI slave init. XFER=%u bytes Telem=%u Cmd=%u",
             (unsigned)SPI_XFER_BYTES, (unsigned)sizeof(telemetry_frame_t), (unsigned)sizeof(command_frame_t));
}

static void spi_telemetry_task(void *arg)
{
    (void)arg;

    uint8_t *tx_dma = heap_caps_malloc(SPI_XFER_BYTES, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(SPI_XFER_BYTES, MALLOC_CAP_DMA);

    if (!tx_dma || !rx_buf)
    {
        ESP_LOGE(TAG_SPI, "DMA buffer alloc failed");
        vTaskDelete(NULL);
    }

    uint32_t seq = 0;
    uint8_t last_mode = CTRL_MODE_DUTY; // Track last mode to detect changes

    while (1)
    {
        telemetry_state_t snap;
        taskENTER_CRITICAL(&telem_mux);
        snap = g_telem_state;
        taskEXIT_CRITICAL(&telem_mux);

        telemetry_frame_t tx;
        memset(&tx, 0, sizeof(tx));
        tx.magic = 0xCAFE;
        tx.version = 1;
        tx.reserved0 = 0;
        tx.seq = seq++;
        memcpy(tx.x, snap.x, sizeof(tx.x));
        memcpy(tx.duty, snap.duty, sizeof(tx.duty));
        tx.crc32 = 0;
        tx.crc32 = esp_crc32_le(0, (const uint8_t *)&tx, sizeof(tx) - sizeof(tx.crc32));

        memset(tx_dma, 0, SPI_XFER_BYTES);
        memcpy(tx_dma, &tx, sizeof(tx));
        memset(rx_buf, 0, SPI_XFER_BYTES);

        spi_slave_transaction_t t = {
            .length = 8 * SPI_XFER_BYTES,
            .tx_buffer = tx_dma,
            .rx_buffer = rx_buf,
        };

        esp_err_t err = spi_slave_transmit(SPI_HOST, &t, portMAX_DELAY);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_SPI, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        // Parse incoming command from first bytes of MOSI
        const command_frame_t *c = (const command_frame_t *)rx_buf;
        if (c->magic == 0xBEEF && c->version == 1 && command_crc_ok(c))
        {
            // Optional: sanity-check mode range before accepting
            if (c->mode <= CTRL_MODE_IDA_PBC)
            {
                // Check if mode changed
                bool mode_changed = (c->mode != last_mode);

                taskENTER_CRITICAL(&cmd_mux);
                g_cmd = *c;
                taskEXIT_CRITICAL(&cmd_mux);

                // Log mode change (outside critical section)
                if (mode_changed)
                {
                    const char *mode_name = (c->mode == CTRL_MODE_DUTY) ? "DUTY" : (c->mode == CTRL_MODE_PI) ? "PI"
                                                                                                             : "IDA-PBC";
                    ESP_LOGI(TAG_SPI, "MODE CHANGED: %s (duty=[%u %u %u])",
                             mode_name, c->duty_cmd[0], c->duty_cmd[1], c->duty_cmd[2]);
                    last_mode = c->mode;
                }
            }
        }
    }
}

// ===== APP MAIN =====
void app_main(void)
{
    // 1. Disable UART0 if driver was installed
    uart_driver_delete(UART_NUM_0);

    // 2. Reset pins to GPIO function
    gpio_reset_pin(GPIO_NUM_1); // U0TXD
    gpio_reset_pin(GPIO_NUM_3); // U0RXD

    mcpwm_unit_t unit = MCPWM_UNIT_0;
    ESP_ERROR_CHECK(mcpwm_setup(unit));
    ESP_LOGI(TAG_PWM, "MCPWM setup complete");

    static mcpwm_task_params_t ctrl_params = {
        .unit = MCPWM_UNIT_0,
        .mode = PWM_MODE_BOOST,
    };

    taskENTER_CRITICAL(&telem_mux);
    memset(&g_telem_state, 0, sizeof(g_telem_state));
    taskEXIT_CRITICAL(&telem_mux);

    spi_slave_init_simple();

    xTaskCreatePinnedToCore(spi_telemetry_task, "spi_telemetry", 4096, NULL, PRIO_SPI, NULL, CORE_SPI);
    xTaskCreatePinnedToCore(mcpwm_control_task, "mcpwm_ctrl", 4096, &ctrl_params, PRIO_CONTROL, NULL, CORE_CONTROL);
    xTaskCreatePinnedToCore(adc_sample_task, "adc", 4096, NULL, PRIO_ADC, NULL, CORE_ADC);
}