/* =============================================================================
 * optim.h  (ANOTADO): interface das rotinas genericas de gradiente/Adam
 * usadas pelo identificador MRAC online em control.c. Ver optim.c.
 * ============================================================================= */

#ifndef OPTIM_H
#define OPTIM_H

float dot_product(const float a[], const float b[], const int n);
void mat_vec_mul(float result[], const float phi[], const float theta[], const int rows, const int cols);
// Gradiente de minimos quadrados para 1 medida escalar (usado na identificacao de R_L/VD por fase).
void compute_gradient_scalar(float grad[], const float phi[], const float y, const float theta[], const int n);
// Gradiente de minimos quadrados ponderado para varias medidas simultaneas (usado na identificacao de carga G/Idc).
void compute_gradient(float grad[], const float phi[], const float y[],
                      const float theta[], const float w[],
                      const int rows, const int cols);
// Passo do otimizador Adam (m/v = momentos de 1a/2a ordem, bias_corr1/2 = correcao de vies).
void adam_update(float *theta, float *m, float *v, const float *grad,
                 const float *alpha, const float beta1, const float beta2, const float eps,
                 const float bias_corr1, const float bias_corr2, const int n);

#endif
