#include "model.h"
#include "config.h"
#include "math.h"

float model_estimate_iload(const float x_hat[], const float duty[], const float x_dot_vout, const float C, const pwm_mode_t mode)
{
    float I_load_est = 0.0f;

    if (mode == PWM_MODE_BOOST)
    {
        for (int i = 0; i < PHASES_COUNT; i++)
        {
            I_load_est += x_hat[i] * (1.0f - duty[i]);
        }
    }
    else
    {
        for (int i = 0; i < PHASES_COUNT; i++)
        {
            I_load_est += x_hat[i];
        }
    }

    I_load_est -= C * x_dot_vout;
    return I_load_est;
}

void model_derivatives(float x_dot_model[], const float x_hat[], const float duty[],
                       const float L[], const float theta_sys[][PARAMS_SYS_COUNT],
                       float C, pwm_mode_t mode)
{
    x_dot_model[PHASES_COUNT] = 0.0f; // reset Vout derivative accumulator

    if (mode == PWM_MODE_BOOST)
    {
        for (int i = 0; i < PHASES_COUNT; i++)
        {
            x_dot_model[i] = (x_hat[SIG_V_IN] - theta_sys[i][IDX_RL] * x_hat[i] - (x_hat[SIG_V_OUT] + theta_sys[i][IDX_VD]) * (1.0f - duty[i])) / L[i];
            x_dot_model[PHASES_COUNT] += x_hat[i] * (1.0f - duty[i]);
        }
    }
    else
    {
        for (int i = 0; i < PHASES_COUNT; i++)
        {
            x_dot_model[i] = ((x_hat[SIG_V_IN] + theta_sys[i][IDX_VD]) * duty[i] - theta_sys[i][IDX_RL] * x_hat[i] - (x_hat[SIG_V_OUT] + theta_sys[i][IDX_VD])) / L[i];
            x_dot_model[PHASES_COUNT] += x_hat[i];
        }
    }

    x_dot_model[PHASES_COUNT] -= x_hat[SIG_I_OUT] / C;
}

void model_power(float P[], const float x_hat[], const float x_dot_model[],
                 const float duty[], const float L[], const float theta_sys[][PARAMS_SYS_COUNT],
                 const float C, const pwm_mode_t mode)
{
    float Vin = x_hat[SIG_V_IN];
    float Vout = x_hat[SIG_V_OUT];
    float Iout = x_hat[SIG_I_OUT];
    float Vout_dot = x_dot_model[PHASES_COUNT];

    P[IDX_P_IN] = 0.0f;
    P[IDX_P_OUT] = Vout * Iout;
    P[IDX_P_R] = 0.0f;
    P[IDX_P_D] = 0.0f;
    P[IDX_P_L] = 0.0f;
    P[IDX_P_C] = C * Vout * Vout_dot;

    for (int i = 0; i < PHASES_COUNT; i++)
    {
        float IL = x_hat[i];
        float IL_dot = x_dot_model[i];

        P[IDX_P_IN] += Vin * IL;
        P[IDX_P_R] += theta_sys[i][IDX_RL] * IL * IL;
        P[IDX_P_L] += L[i] * IL * IL_dot;

        if (mode == PWM_MODE_BOOST)
            P[IDX_P_D] += theta_sys[i][IDX_VD] * (1.0f - duty[i]) * IL;
        else
            P[IDX_P_D] += theta_sys[i][IDX_VD] * (1.0f - duty[i]) * IL; // D_top = 1 - D_bot for synchronous
    }
}

float compute_IL_eq(const float theta_sys[][2], const float theta_load[],
                    const float x_hat[], const float Vout_ref)
{
    float I_load_eq = theta_load[IDX_G] * Vout_ref + theta_load[IDX_IDC];

    float a = 0.0f;
    float b = 0.0f;
    for (int i = 0; i < PHASES_COUNT; i++)
    {
        float denom = theta_sys[i][IDX_VD] + Vout_ref;
        a += theta_sys[i][IDX_RL] / denom;
        b -= x_hat[SIG_V_IN] / denom;
    }

    float disc_eq = b * b - 4.0f * a * I_load_eq;
    float disc_meas = b * b - 4.0f * a * x_hat[SIG_I_OUT];

    if (disc_eq > 0.0f)
    {
        return (-b - sqrtf(disc_eq)) / (2.0f * a);
    }
    else if (disc_meas > 0.0f)
    {
        return (-b - sqrtf(disc_meas)) / (2.0f * a);
    }

    return 0.0f;
}