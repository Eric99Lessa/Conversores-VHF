/* =============================================================================
 * optim.c  (ANOTADO)
 * =============================================================================
 * Rotinas numericas GENERICAS de otimizacao por gradiente (nao conhecem
 * nada sobre conversores CC-CC): produto escalar, multiplicacao
 * matriz-vetor, gradiente de minimos quadrados (escalar e matricial) e o
 * otimizador Adam. Sao usadas por control.c para implementar o
 * identificador MRAC online de R_L, V_D (por fase) e do modelo de carga
 * (G, Idc): ver os comentarios em control.c, secao "IDENTIFICACAO ONLINE".
 * ============================================================================= */

#include "optim.h"
#include "math.h"

// Produto escalar de dois vetores de tamanho n.
float dot_product(const float a[], const float b[], const int n)
{
    float result = 0.0f;
    for (int j = 0; j < n; j++)
    {
        result += a[j] * b[j];
    }
    return result;
}

// y = phi * theta, com phi (rows x cols) armazenada em row-major e theta (cols x 1).
void mat_vec_mul(float result[], const float phi[], const float theta[], const int rows, const int cols)
{
    for (int i = 0; i < rows; i++)
    {
        result[i] = dot_product(&phi[i * cols], theta, cols);
    }
}

/* compute_gradient_scalar(): gradiente do erro quadratico e = (y - phi.theta)^2 em relacao a theta, para o caso
 * de UMA UNICA medida escalar y (regressao linear com 1 equacao, n incognitas). grad[j] = -2*e*phi[j]/2 = e*phi[j]
 * (o -2 e absorvido na taxa de aprendizado do Adam, que ja e escolhida empiricamente). */
void compute_gradient_scalar(float grad[], const float phi[], const float y, const float theta[], const int n)
{
    float res = y - dot_product(phi, theta, n); // residuo (erro de predicao)
    for (int j = 0; j < n; j++)
    {
        grad[j] = res * phi[j];
    }
}

/* compute_gradient(): mesma ideia, mas para VARIAS medidas y[] simultaneas (rows equacoes, cols incognitas),
 * cada uma ponderada por w[i]: usado no identificador de carga em control.c, que combina o valor e a derivada
 * de Vout/Iout como duas "medidas" com pesos diferentes (w_load = {1.0, 0.0003}). */
void compute_gradient(float grad[], const float phi[], const float y[],
                      const float theta[], const float w[],
                      const int rows, const int cols)
{
    float y_hat[rows];
    mat_vec_mul(y_hat, phi, theta, rows, cols); // predicao atual para cada uma das "rows" medidas

    for (int j = 0; j < cols; j++)
    {
        grad[j] = 0.0f;
        for (int i = 0; i < rows; i++)
        {
            float res = y[i] - y_hat[i];
            grad[j] += w[i] * res * phi[i * cols + j]; // soma ponderada da contribuicao de cada medida
        }
    }
}

/* adam_update(): implementacao padrao do otimizador Adam (Kingma & Ba, 2015): momento de 1a ordem (m, media
 * movel do gradiente) e 2a ordem (v, media movel do gradiente ao quadrado), cada um com sua propria correcao de
 * vies (bias_corr1/2, calculada em control.c em funcao do "tempo" decorrido). alpha[] e o vetor de taxas de
 * aprendizado (uma por parametro, ja escalada por Ts em control.c). Isto e o que torna a identificacao de R_L/VD
 * e G/Idc um processo ADAPTATIVO continuo (MRAC), em vez de uma estimativa unica offline. */
void adam_update(float *theta, float *m, float *v, const float *grad,
                 const float *alpha, const float beta1, const float beta2, const float eps,
                 const float bias_corr1, const float bias_corr2, const int n)
{
    for (int j = 0; j < n; j++)
    {
        m[j] = beta1 * m[j] + (1.0f - beta1) * grad[j];                             // media movel do gradiente (momento)
        v[j] = beta2 * v[j] + (1.0f - beta2) * (grad[j] - m[j]) * (grad[j] - m[j]);  // media movel da variancia do gradiente

        float m_hat = m[j] / bias_corr1; // correcao de vies (m e v comecam em 0, enviesados no inicio)
        float v_hat = v[j] / bias_corr2;

        theta[j] += alpha[j] * m_hat / (sqrtf(v_hat) + eps); // passo adaptativo (maior onde o gradiente e mais estavel)
    }
}
