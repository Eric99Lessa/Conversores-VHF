#ifndef OPTIM_H
#define OPTIM_H

float dot_product(const float a[], const float b[], const int n);
void mat_vec_mul(float result[], const float phi[], const float theta[], const int rows, const int cols);
void compute_gradient_scalar(float grad[], const float phi[], const float y, const float theta[], const int n);
void compute_gradient(float grad[], const float phi[], const float y[],
                      const float theta[], const float w[],
                      const int rows, const int cols);
void adam_update(float *theta, float *m, float *v, const float *grad,
                 const float *alpha, const float beta1, const float beta2, const float eps,
                 const float bias_corr1, const float bias_corr2, const int n);

#endif