#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG_PWM = "MCPWM";

#define PWM_FREQUENCY_HZ 20000
#define PWM_INIT_DUTY_PC 0.0f

#define U_L_GPIO (22)
#define V_L_GPIO (21)
#define W_L_GPIO (5)
#define U_U_GPIO (23)
#define V_U_GPIO (3) // NOTE: GPIO3 is UART0 RX (not recommended in final HW)
#define W_U_GPIO (18)

#define U_ERR_GPIO (25)
#define V_ERR_GPIO (19)
#define W_ERR_GPIO (17)

typedef enum
{
    PWM_MODE_BOOST = 0,
    PWM_MODE_BUCK = 1,
} pwm_mode_t;

static inline float clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : (x > hi) ? hi
                                    : x;
}

static esp_err_t mcpwm_setup(mcpwm_unit_t unit)
{
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM0A, U_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM0B, U_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM1A, V_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM1B, V_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM2A, W_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(unit, MCPWM2B, W_U_GPIO));

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

    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F0));
    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F1));
    ESP_ERROR_CHECK(mcpwm_fault_init(unit, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F2));

    mcpwm_output_action_t action_a = MCPWM_ACTION_FORCE_LOW;
    mcpwm_output_action_t action_b = MCPWM_ACTION_FORCE_LOW;

    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_0, MCPWM_SELECT_F0, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_1, MCPWM_SELECT_F1, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(unit, MCPWM_TIMER_2, MCPWM_SELECT_F2, action_a, action_b));

    return ESP_OK;
}

static void update_duty(mcpwm_unit_t unit, float duty_u, float duty_v, float duty_w, pwm_mode_t mode)
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

static void pwm_ramp_task(void *arg)
{
    (void)arg;
    const mcpwm_unit_t unit = MCPWM_UNIT_0;
    const pwm_mode_t mode = PWM_MODE_BOOST;

    float duty = 0.0f;
    float step = 1.0f;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(50);

    while (1)
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

        vTaskDelayUntil(&last, period);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(mcpwm_setup(MCPWM_UNIT_0));
    ESP_LOGI(TAG_PWM, "PWM test running (20kHz), ramping duty.");
    xTaskCreate(pwm_ramp_task, "pwm_ramp", 4096, NULL, 5, NULL);
}