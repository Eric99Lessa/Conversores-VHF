/* =============================================================================
 * observer.h  (ANOTADO): interface do filtro de Kalman escalar (modelo
 * cinematico valor+derivada). Ver observer.c para a explicacao completa.
 * ============================================================================= */

#ifndef OBSERVER_H
#define OBSERVER_H

// Passo de atualizacao (correcao pela medida z[]) para "count" grandezas independentes.
void kf_update_states(const float x_pred[], const float x_dot_pred[], float p11[], float p12[], float p22[],
                      const float z[], const float R[], float x_val[], float x_dot[], const int count);

// Passo de predicao (propagacao no tempo, integracao de Euler com passo Ts) para "count" grandezas independentes.
void kf_predict_states(float x_pred[], float x_dot_pred[], const float x_val[], const float x_dot[],
                       float p11[], float p12[], float p22[], const float q_val[], const float q_dot[], const float Ts, const int count);

#endif
