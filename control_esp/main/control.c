#include "control.h"
#include "config.h"
#include "adc.h"
#include "observer.h"
#include "model.h"
#include "optim.h"
#include "math.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "string.h"

_Static_assert(ADC_MEAS_IL_OUT == 0 || ADC_MEAS_IL_OUT == 1,
               "ADC_MEAS_IL_OUT must be 0 or 1");

static esp_err_t mcpwm_setup(void)
{
    // GPIO INIT PWM
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM0A, U_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM0B, U_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM1A, V_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM1B, V_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM2A, W_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM2B, W_U_GPIO));

    // GPIO INIT FAULT
    gpio_config_t fault_gpio_conf = {
        .pin_bit_mask = (1ULL << U_ERR_GPIO) | (1ULL << V_ERR_GPIO) | (1ULL << W_ERR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&fault_gpio_conf));

    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM_FAULT_0, U_ERR_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM_FAULT_1, V_ERR_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM_FAULT_2, W_ERR_GPIO));

    // Frequency and initial value
    mcpwm_config_t pwm_cfg = {
        .frequency = PWM_FREQUENCY_HZ,
        .cmpr_a = PWM_INIT_DUTY_PC,
        .cmpr_b = PWM_INIT_DUTY_PC,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, MCPWM_TIMER_0, &pwm_cfg));
    ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, MCPWM_TIMER_1, &pwm_cfg));
    ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, MCPWM_TIMER_2, &pwm_cfg));

    mcpwm_set_timer_sync_output(PWM_UNIT, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);

    mcpwm_sync_config_t sync_cfg = {
        .sync_sig = MCPWM_SELECT_TIMER0_SYNC,
        .timer_val = 0,
        .count_direction = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ESP_ERROR_CHECK(mcpwm_sync_configure(PWM_UNIT, MCPWM_TIMER_0, &sync_cfg));
    sync_cfg.timer_val = 333;
    ESP_ERROR_CHECK(mcpwm_sync_configure(PWM_UNIT, MCPWM_TIMER_1, &sync_cfg));
    sync_cfg.timer_val = 667;
    ESP_ERROR_CHECK(mcpwm_sync_configure(PWM_UNIT, MCPWM_TIMER_2, &sync_cfg));

    ESP_ERROR_CHECK(mcpwm_fault_init(PWM_UNIT, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F0));
    ESP_ERROR_CHECK(mcpwm_fault_init(PWM_UNIT, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F1));
    ESP_ERROR_CHECK(mcpwm_fault_init(PWM_UNIT, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F2));

    mcpwm_output_action_t action_a = MCPWM_ACTION_FORCE_LOW;
    mcpwm_output_action_t action_b = MCPWM_ACTION_FORCE_LOW;

    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(PWM_UNIT, MCPWM_TIMER_0, MCPWM_SELECT_F0, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(PWM_UNIT, MCPWM_TIMER_1, MCPWM_SELECT_F1, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(PWM_UNIT, MCPWM_TIMER_2, MCPWM_SELECT_F2, action_a, action_b));

    return ESP_OK;
}

// Update duty
static void update_duty(const float duty[3], pwm_mode_t mode)
{
    if (mode == PWM_MODE_BOOST)
    {
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty[0]);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, duty[1]);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, duty[2]);
    }
    else
    {
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, duty[0]);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, duty[1]);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, duty[2]);
    }
}

// ===== CONTROL MODE FUNCTIONS =====

static void duty_open(float duty_out[3], float duty_in[3])
{
    duty_out[0] = duty_in[0];
    duty_out[1] = duty_in[1];
    duty_out[2] = duty_in[2];
}

static void duty_cascade(float duty_out[3], float x[SIG_COUNT], const float L[], float theta_sys[][PARAMS_SYS_COUNT], float IL_ref, float Kp_i)
{
    // 1 - (Vin-IL.*R_L+Kp_i.*L.*(IL-IL_ref))./(VD+Vout)

    // Get states
    float Vin = x[SIG_V_IN];
    float Vout = x[SIG_V_OUT];

    for (int i = 0; i < PHASES_COUNT; i++)
    {
        duty_out[i] = 1 - (Vin - x[i] * theta_sys[i][IDX_RL] + Kp_i * L[i] * (x[i] - IL_ref)) / (theta_sys[i][IDX_VD] + Vout);
    }
}

void clamp_array(float theta[], const float theta_min[], const float theta_max[], int n)
{
    for (int j = 0; j < n; j++)
    {
        theta[j] = fmaxf(fminf(theta[j], theta_max[j]), theta_min[j]);
    }
}

static const char *TAG_PWM = "MCPWM";

// ===== CONTROL TASK =====
void control_task(void *arg)
{
    task_params_t *params = (task_params_t *)arg;

    ESP_ERROR_CHECK(mcpwm_setup());
    ESP_LOGI(TAG_PWM, "MCPWM setup complete");

    TickType_t last_wake_time = xTaskGetTickCount();

    adc1_config_width(ADC_WIDTH_CFG);
    for (int i = 0; i < SIG_COUNT; i++)
    {
        adc1_config_channel_atten(adc_sig_channel[i], ADC_ATTEN_CFG);
    }

    const float Ts = 1.0f / CONTROL_RATE_HZ;

    const float C = C_nom;
    float L[PHASES_COUNT] = {0.0};
    float theta_sys[PHASES_COUNT][2] = {0};
    float m_sys[PHASES_COUNT][2] = {0};
    float v_sys[PHASES_COUNT][2] = {0};
    float theta_load[2] = {0};
    const float theta_load_min[PARAMS_LOAD_COUNT] = {0.0f, 0.0f};
    const float theta_load_max[PARAMS_LOAD_COUNT] = {G_LOAD_MAX, IDC_LOAD_MAX};
    float m_load[2] = {0};
    float v_load[2] = {0};

    const float alpha_sys[PARAMS_SYS_COUNT] = {0.08f * Ts, 0.1f * Ts};
    const float beta1_sys = 0.94f;
    const float beta2_sys = 0.999f;
    const float alpha_load[PARAMS_LOAD_COUNT] = {1.0f * Ts, 100.0f * Ts};
    const float beta1_load = 0.94f;
    const float beta2_load = 0.99f;
    int adam_step = 0;

    const float theta_sys_min[PARAMS_SYS_COUNT] = {RL_MIN, VD_MIN};
    const float theta_sys_max[PARAMS_SYS_COUNT] = {RL_MAX, VD_MAX};
    for (int i = 0; i < PHASES_COUNT; i++)
    {
        L[i] = L_nom;
        theta_sys[i][IDX_RL] = RL_nom;
        theta_sys[i][IDX_VD] = VD_nom;
    }

    float z[SIG_COUNT] = {0.0}; // Uses SIG_COUNT instead of Z_COUNT due to pseudo-measurement
    float x_pred[SIG_COUNT] = {0.0};
    float x_dot_pred[SIG_COUNT] = {0.0};
    float x_hat[SIG_COUNT] = {0.0};
    float x_dot_hat[SIG_COUNT] = {0.0};
    float x_dot_model[PHASES_COUNT + 1] = {0.0}; // 1 for each current and 1 for output voltage
    float duty[PHASES_COUNT] = {0.0f};
    float duty_bot[PHASES_COUNT] = {0.0f};
    float duty_top[PHASES_COUNT] = {0.0f};
    float int_eV = 0.0f;

    float p11[SIG_COUNT] = {1.0};
    float p12[SIG_COUNT] = {0.0};
    float p22[SIG_COUNT] = {1.0};
    const float R[SIG_COUNT] = R_MEAS;
    const float Q_val[SIG_COUNT] = Q_VAL;
    const float Q_dot[SIG_COUNT] = Q_DOT;

    float Kp_i = 1;
    float Kp_v = 1;
    float Ki_v = 1;

    while (1)
    {
        // Measurements
        adc_read_states(z, Z_COUNT);

        // Update
        if (Z_COUNT < SIG_COUNT) // I_load is not measured, so we estimate it
        {
            z[SIG_I_OUT] = model_estimate_iload(x_hat, duty, x_dot_hat[SIG_V_OUT], C, params->pwm_mode);
        }
        kf_update_states(x_pred, x_dot_pred, p11, p12, p22, z, R, x_hat, x_dot_hat, SIG_COUNT);

        // Evaluate derivatives by model
        model_derivatives(x_dot_model, x_hat, duty, L, theta_sys, C, params->pwm_mode);

        // MRAC based (using ADAM algorithm) update for RL and VD
        adam_step++;
        float bias_corr1_sys = 1.0f - powf(beta1_sys, adam_step * Ts);
        float bias_corr2_sys = 1.0f - powf(beta2_sys, adam_step * Ts);
        float bias_corr1_load = 1.0f - powf(beta1_load, adam_step * Ts);
        float bias_corr2_load = 1.0f - powf(beta2_load, adam_step * Ts);

        for (int i = 0; i < PHASES_COUNT; i++)
        {
            // Residual
            float phi_IL[PARAMS_SYS_COUNT] = {x_hat[i], (1.0f - duty_bot[i])};
            float y_IL = x_hat[SIG_V_IN] - x_hat[SIG_V_OUT] * (1.0f - duty_bot[i]) - L[i] * x_dot_model[i];

            // Gradient
            float grad[PARAMS_SYS_COUNT];
            compute_gradient_scalar(grad, phi_IL, y_IL, theta_sys[i], PARAMS_SYS_COUNT);

            // Adam update
            adam_update(theta_sys[i], m_sys[i], v_sys[i], grad,
                        alpha_sys, beta1_sys, beta2_sys, 1e-8f,
                        bias_corr1_sys, bias_corr2_sys, PARAMS_SYS_COUNT);

            // Clamp
            clamp_array(theta_sys[i], theta_sys_min, theta_sys_max, PARAMS_SYS_COUNT);
        }

        // Residual
        float phi_load[2][PARAMS_LOAD_COUNT] = {
            {x_hat[SIG_V_OUT], 1.0f},
            {x_dot_hat[SIG_V_OUT], 0.0f}};

        float y_load[2] = {
            x_hat[SIG_I_OUT],
            x_dot_hat[SIG_I_OUT]};

        float w_load[PARAMS_LOAD_COUNT] = {1.0f, 0.0003f};
        float grad[PARAMS_LOAD_COUNT];
        compute_gradient(grad, (const float *)phi_load, y_load, theta_load, w_load, 2, PARAMS_LOAD_COUNT);

        // Adam update
        adam_update(theta_load, m_load, v_load, grad,
                    alpha_load, beta1_load, beta2_load, 1e-8f,
                    bias_corr1_load, bias_corr2_load, PARAMS_LOAD_COUNT);

        // Clamp
        clamp_array(theta_load, theta_load_min, theta_load_max, PARAMS_LOAD_COUNT);

#if OPERATION_MODE == OP_MODE_STANDALONE
        float IL_eq = compute_IL_eq(theta_sys, theta_load, x_hat, params->Vout_ref);

        float err_V = params->Vout_ref - x_hat[SIG_V_OUT];
        int_eV += Ts * err_V;

        float IL_ref = IL_eq + Kp_v * err_V + Ki_v * int_eV;
#else
        float IL_ref = params->IL_ref;
#endif

        // Compute duty based on selected mode
        switch ((control_mode_t)params->ctrl_mode)
        {
        case CTRL_OPEN:
        {
            float d_cmd[PHASES_COUNT];
            taskENTER_CRITICAL(&params->telem_mux);
            memcpy(d_cmd, (const void *)params->d_cmd, sizeof(d_cmd));
            taskEXIT_CRITICAL(&params->telem_mux);
            duty_open(duty, d_cmd);
            break;
        }
        case CTRL_CASCADE:
            duty_cascade(duty, x_hat, L, theta_sys, IL_ref, Kp_i);
            break;
        default:
            // Unknown mode -> Keep at 0
            break;
        }

        // Apply duty
        update_duty(duty, params->pwm_mode);

        if (params->pwm_mode == PWM_MODE_BOOST)
        {
            memcpy(duty_bot, duty, sizeof(duty));
            memset(duty_top, 0, sizeof(duty_top));
        }
        else
        {
            memset(duty_bot, 0, sizeof(duty_bot));
            memcpy(duty_top, duty, sizeof(duty));
        }

        // Predict
        // Note that the prediction step does not depend on Z, so it's the same for load current even if it's not measured
        kf_predict_states(x_pred, x_dot_pred, x_hat, x_dot_hat, p11, p12, p22, Q_val, Q_dot, Ts, SIG_COUNT);

        taskENTER_CRITICAL(&params->telem_mux);
        memcpy((void *)params->x, x_hat, sizeof(x_hat));
        memcpy((void *)params->x_pred, x_pred, sizeof(x_pred));
        memcpy((void *)params->x_dot, x_dot_hat, sizeof(x_dot_hat));
        memcpy((void *)params->x_dot_model, x_dot_model, sizeof(x_dot_model));
        memcpy((void *)params->theta_sys, theta_sys, sizeof(theta_sys));
        memcpy((void *)params->theta_load, theta_load, sizeof(theta_load));
        memcpy((void *)params->d_bot, duty_bot, sizeof(duty_bot));
        memcpy((void *)params->d_top, duty_top, sizeof(duty_top));
        taskEXIT_CRITICAL(&params->telem_mux);

        // Delay
        vTaskDelayUntil(&last_wake_time, CONTROL_PERIOD_TICKS);
    }
}