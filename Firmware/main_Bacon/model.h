/* =============================================================================
 * model.h  (ANOTADO): interface do modelo medio do conversor. Ver model.c
 * para a explicacao completa de cada funcao e as ressalvas [REVISAR].
 * ============================================================================= */

#ifndef MODEL_H
#define MODEL_H

#include "config.h"

// Estima I_out (corrente de carga) por balanco de corrente, quando ADC_MEAS_IL_OUT==0 (config.h).
float model_estimate_iload(const float x_hat[], const float duty[], const float x_dot_vout, const float C, const pwm_mode_t mode);

// Deriva dIL/dt (por fase) e dVout/dt a partir do modelo medio (boost ou buck) e dos parametros theta_sys atuais.
void model_derivatives(float x_dot_model[], const float x_hat[], const float duty[],
                       const float L[], const float theta_sys[][PARAMS_SYS_COUNT],
                       const float C, const pwm_mode_t mode);

// Balanco de potencia (entrada/saida/perdas): so para telemetria/diagnostico, nao usado na malha de controle.
void model_power(float P[], const float x_hat[], const float x_dot_model[],
                 const float duty[], const float L[], const float theta_sys[][PARAMS_SYS_COUNT],
                 const float C, const pwm_mode_t mode);

// Corrente de indutor de equilibrio (feedforward nao-linear do laco de tensao STANDALONE).
// [REVISAR: item #5 do relatorio] mal-condicionada para R_L pequeno, ver model.c.
float compute_IL_eq(const float theta_sys[][2], const float theta_load[],
                    const float x_hat[], const float Vout_ref);

#endif
