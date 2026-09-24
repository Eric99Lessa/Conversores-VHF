/* =============================================================================
 * observer.c  (ANOTADO)
 * =============================================================================
 * Filtro de Kalman ESCALAR (um filtro independente por grandeza medida),
 * com modelo de estado "valor + derivada constante entre amostras" (as
 * vezes chamado de modelo alpha-beta ou "constant velocity" na literatura
 * de rastreamento). Para cada grandeza i (corrente de fase, Vin, Vout,
 * Iout) o estado e [x_i, x_dot_i] com covariancia [p11_i, p12_i; p12_i,
 * p22_i] (matriz 2x2 simetrica, guardada nos 3 arrays p11/p12/p22).
 *
 * Isso e DIFERENTE do observador de estados do artigo CBA2026 (que usa o
 * modelo de espaco de estados fisico do conversor, eqs. discrete_model,
 * Ad=I+Ts*A / Bd=Ts*B, para PREDIZER IL e Vout a partir da entrada de
 * controle). Aqui o modelo de predicao e puramente CINEMATICO (nao usa
 * Vin, Vout, duty nem R_L/L/C para prever x_dot: assume que a derivada
 * observada no ciclo anterior continua valendo, corrigida so pela nova
 * medicao). E um filtro generico de suavizacao/derivacao, nao um observador
 * baseado no modelo fisico da planta. O modelo FISICO (model.c) e usado
 * separadamente em control.c, so para calcular o residuo do identificador
 * MRAC, nao para corrigir x_hat aqui.
 * ============================================================================= */

#include "observer.h"
#include "config.h"

/* -----------------------------------------------------------------------
 * kf_update_scalar(): passo de ATUALIZACAO (correcao) do Kalman escalar
 * para uma unica grandeza. Formulas classicas de Kalman com H=[1,0]
 * (so o valor e medido diretamente, nao a derivada):
 *   inovacao = medida - predito
 *   S = p11 + R          (covariancia da inovacao)
 *   K = [p11; p12] / S   (ganho de Kalman, um por estado)
 *   x_hat = x_pred + K*inovacao
 *   P = P - K*H*P        (atualizacao de covariancia, forma reduzida abaixo)
 * ----------------------------------------------------------------------- */
static void kf_update_scalar(float *x_val, float *x_dot, float *p11, float *p12, float *p22, const float x_pred, const float x_dot_pred, const float z, const float R)
{
    float innov = z - x_pred;
    float S = *p11 + R;  // covariancia da inovacao (incerteza da predicao + ruido de medicao)
    float k1 = *p11 / S; // ganho de Kalman para o VALOR
    float k2 = *p12 / S; // ganho de Kalman para a DERIVADA (via correlacao cruzada p12)

    *x_val = x_pred + k1 * innov;
    *x_dot = x_dot_pred + k2 * innov;

    // Atualizacao de covariancia (ordem importa: p22 e p12 usam o p12 ANTIGO, entao sao atualizados antes de p11).
    *p22 -= k2 * (*p12);
    *p12 -= k1 * (*p12);
    *p11 -= k1 * (*p11);
}

/* -----------------------------------------------------------------------
 * kf_predict_scalar(): passo de PREDICAO (propagacao) do Kalman escalar.
 * Modelo de processo: x[k+1] = x[k] + Ts*x_dot[k] (integracao de Euler),
 * x_dot[k+1] = x_dot[k] (derivada assumida constante entre amostras, com
 * incerteza adicionada por q_dot). q_val e q_dot sao os ruidos de processo
 * (config.h: Q_VAL, Q_DOT): Q_DOT >> Q_VAL indica pouca confianca na
 * hipotese de derivada constante, o que e razoavel num conversor chaveado.
 * ----------------------------------------------------------------------- */
static void kf_predict_scalar(float *x_val_p, float *x_dot_p, float *p11, float *p12, float *p22,
                              const float x_val, const float x_dot, const float q_val, const float q_dot, const float Ts)
{
    // Propagacao do estado f(x): modelo cinematico de derivada constante.
    *x_dot_p = x_dot;
    *x_val_p = x_val + Ts * x_dot;

    // Propagacao de covariancia: P[k+1] = F*P[k]*F' + Q, com F = [[1, Ts],[0, 1]] (Jacobiano do modelo acima).
    *p11 += 2 * Ts * (*p12) + (*p22) * Ts * Ts + q_val;
    *p12 += Ts * (*p22);
    *p22 += q_dot;
}

// Aplica kf_update_scalar a cada uma das "count" grandezas independentes (sem acoplamento entre elas).
void kf_update_states(const float x_pred[], const float x_dot_pred[], float p11[], float p12[], float p22[], const float z[], const float R[], float x_val[], float x_dot[], const int count)
{
    for (int i = 0; i < count; i++)
    {
        kf_update_scalar(&x_val[i], &x_dot[i], &p11[i], &p12[i], &p22[i], x_pred[i], x_dot_pred[i], z[i], R[i]);
    }
}

// Aplica kf_predict_scalar a cada uma das "count" grandezas independentes.
void kf_predict_states(float x_pred[], float x_dot_pred[], const float x_val[], const float x_dot[], float p11[], float p12[], float p22[],
                       const float q_val[], const float q_dot[], const float Ts, const int count)
{
    for (int i = 0; i < count; i++)
    {
        kf_predict_scalar(&x_pred[i], &x_dot_pred[i], &p11[i], &p12[i], &p22[i], x_val[i], x_dot[i], q_val[i], q_dot[i], Ts);
    }
}
