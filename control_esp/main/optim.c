#include "optim.h"
#include "math.h"

float dot_product(const float a[], const float b[], const int n)
{
    float result = 0.0f;
    for (int j = 0; j < n; j++)
    {
        result += a[j] * b[j];
    }
    return result;
}

void mat_vec_mul(float result[], const float phi[], const float theta[], const int rows, const int cols)
{
    for (int i = 0; i < rows; i++)
    {
        result[i] = dot_product(&phi[i * cols], theta, cols);
    }
}

// y is scalar, phi is 1D (1 x n)
void compute_gradient_scalar(float grad[], const float phi[], const float y, const float theta[], const int n)
{
    float res = y - dot_product(phi, theta, n);
    for (int j = 0; j < n; j++)
    {
        grad[j] = res * phi[j];
    }
}

// y is array, phi is 2D matrix (rows x cols) stored row-major
void compute_gradient(float grad[], const float phi[], const float y[],
                      const float theta[], const float w[],
                      const int rows, const int cols)
{
    float y_hat[rows];
    mat_vec_mul(y_hat, phi, theta, rows, cols);

    for (int j = 0; j < cols; j++)
    {
        grad[j] = 0.0f;
        for (int i = 0; i < rows; i++)
        {
            float res = y[i] - y_hat[i];
            grad[j] += w[i] * res * phi[i * cols + j];
        }
    }
}

void adam_update(float *theta, float *m, float *v, const float *grad,
                 const float *alpha, const float beta1, const float beta2, const float eps,
                 const float bias_corr1, const float bias_corr2, const int n)
{
    for (int j = 0; j < n; j++)
    {
        m[j] = beta1 * m[j] + (1.0f - beta1) * grad[j];
        v[j] = beta2 * v[j] + (1.0f - beta2) * (grad[j] - m[j]) * (grad[j] - m[j]);

        float m_hat = m[j] / bias_corr1;
        float v_hat = v[j] / bias_corr2;

        theta[j] += alpha[j] * m_hat / (sqrtf(v_hat) + eps);
    }
}