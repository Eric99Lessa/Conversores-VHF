#ifndef MODEL_H
#define MODEL_H

#include "config.h"

float model_estimate_iload(const float x_hat[], const float duty[], const float x_dot_vout, const float C, const pwm_mode_t mode);
void model_derivatives(float x_dot_model[], const float x_hat[], const float duty[],
                       const float L[], const float theta_sys[][PARAMS_SYS_COUNT],
                       const float C, const pwm_mode_t mode);
void model_power(float P[], const float x_hat[], const float x_dot_model[],
                 const float duty[], const float L[], const float theta_sys[][PARAMS_SYS_COUNT],
                 const float C, const pwm_mode_t mode);
float compute_IL_eq(const float theta_sys[][2], const float theta_load[],
                    const float x_hat[], const float Vout_ref);

#endif