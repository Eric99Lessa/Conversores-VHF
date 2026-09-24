/* =============================================================================
 * model.c  (ANOTADO)
 * =============================================================================
 * Bloco numerico puro (sem I/O) com o modelo medio do conversor CC-CC de
 * 3 fases entrelacadas (boost ou buck). E usado por control.c para:
 *   - prever a derivada da corrente/tensao a cada ciclo (model_derivatives),
 *     comparada com a derivada medida pelo observador para gerar o residuo
 *     do identificador MRAC/Adam;
 *   - estimar I_out quando ele nao e medido por ADC (model_estimate_iload);
 *   - calcular o balanco de potencia para telemetria/diagnostico (model_power);
 *   - resolver a corrente de indutor de equilibrio que sustenta Vout_ref em
 *     regime permanente (compute_IL_eq), usada como feedforward no laco de
 *     tensao STANDALONE em control.c.
 *
 * Nenhuma destas funcoes conhece Ts nem periodo de amostragem: trabalham
 * inteiramente em tempo continuo (derivadas, nao incrementos). A
 * discretizacao (quando existe) acontece do lado de fora, em control.c
 * (ex.: kf_predict_states integra x_dot por Euler-avante usando Ts).
 * ============================================================================= */

#include "model.h"
#include "config.h"
#include "math.h"

/* -----------------------------------------------------------------------
 * model_estimate_iload(): estima I_out (corrente de carga) quando ela nao e
 * medida por ADC (ADC_MEAS_IL_OUT == 0, config.h). Usa o balanco de
 * corrente no no de saida: a soma das correntes de fase que efetivamente
 * chegam ao barramento de saida (IL*(1-duty) no boost, ou IL direto no
 * buck) menos a corrente que esta carregando/descarregando o capacitor
 * (C*dVout/dt) e igual a corrente de carga.
 * ----------------------------------------------------------------------- */
float model_estimate_iload(const float x_hat[], const float duty[], const float x_dot_vout, const float C, const pwm_mode_t mode)
{
    float I_load_est = 0.0f;

    if (mode == PWM_MODE_BOOST)
    {
        // No boost, a corrente que chega ao barramento de saida por cada fase e IL*(1-D) (o diodo/sincrono de
        // cima conduz durante a fracao (1-D) do periodo).
        for (int i = 0; i < PHASES_COUNT; i++)
        {
            I_load_est += x_hat[i] * (1.0f - duty[i]);
        }
    }
    else
    {
        // No buck, a corrente de cada fase chega integralmente ao no de saida (a reducao de tensao acontece do
        // lado da entrada, nao aqui).
        for (int i = 0; i < PHASES_COUNT; i++)
        {
            I_load_est += x_hat[i];
        }
    }

    I_load_est -= C * x_dot_vout; // subtrai a corrente que esta indo para o capacitor (nao para a carga)
    return I_load_est;
}

/* -----------------------------------------------------------------------
 * model_derivatives(): implementa as EDOs medias do conversor (a mesma
 * familia de equacoes usada no artigo CBA2026, eqs. edo_i_continuos /
 * edo_v_continuos, mas aqui por fase e com R_L, V_D estimados online em vez
 * de fixos):
 *   Boost: L*dIL/dt = Vin - IL*R_L - (Vout+VD)*(1-D)
 *   Buck:  L*dIL/dt = (Vin+VD)*D - IL*R_L - (Vout+VD)
 * dVout/dt (x_dot_model[PHASES_COUNT]) acumula a contribuicao de cada fase
 * menos a corrente de carga medida/estimada (x_hat[SIG_I_OUT]), dividida
 * pela capacitancia C.
 * ----------------------------------------------------------------------- */
void model_derivatives(float x_dot_model[], const float x_hat[], const float duty[],
                       const float L[], const float theta_sys[][PARAMS_SYS_COUNT],
                       float C, pwm_mode_t mode)
{
    x_dot_model[PHASES_COUNT] = 0.0f; // zera o acumulador de dVout/dt antes de somar a contribuicao de cada fase

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

    x_dot_model[PHASES_COUNT] -= x_hat[SIG_I_OUT] / C; // fecha o balanco no capacitor: dVout/dt = (soma IL_eff - Iout)/C
}

/* -----------------------------------------------------------------------
 * model_power(): calcula o balanco de potencia instantaneo (entrada, saida,
 * perdas resistivas, perdas de diodo, indutiva, capacitiva): usado so para
 * telemetria/diagnostico em bancada, nao entra na malha de controle.
 * [REVISAR] o ramo boost e buck de P_D calculam exatamente a mesma
 * expressao (o comentario do proprio autor original ja observa isso:
 * "D_top = 1 - D_bot for synchronous"): nao e um bug funcional, mas o
 * if/else e redundante e pode ser simplificado.
 * ----------------------------------------------------------------------- */
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
    P[IDX_P_C] = C * Vout * Vout_dot; // potencia armazenada/liberada pelo capacitor de saida

    for (int i = 0; i < PHASES_COUNT; i++)
    {
        float IL = x_hat[i];
        float IL_dot = x_dot_model[i];

        P[IDX_P_IN] += Vin * IL;                          // potencia entregue pela fonte de entrada, por fase
        P[IDX_P_R] += theta_sys[i][IDX_RL] * IL * IL;      // perda resistiva no indutor (R_L * IL^2), por fase
        P[IDX_P_L] += L[i] * IL * IL_dot;                  // potencia instantanea armazenada no indutor (L*IL*dIL/dt)

        if (mode == PWM_MODE_BOOST)
            P[IDX_P_D] += theta_sys[i][IDX_VD] * (1.0f - duty[i]) * IL;
        else
            P[IDX_P_D] += theta_sys[i][IDX_VD] * (1.0f - duty[i]) * IL; // D_top = 1 - D_bot for synchronous
    }
}

/* -----------------------------------------------------------------------
 * compute_IL_eq(): resolve, para CADA amostra, a corrente de indutor total
 * (dividida igualmente entre as 3 fases nesta formulacao escalar) que, em
 * REGIME PERMANENTE, sustentaria Vout_ref dado o modelo de carga estimado
 * I_load_eq = G*Vout_ref + Idc. Vem da equacao de balanco de potencia do
 * boost em regime (Vin*IL = R_L*IL^2 + VD*(1-D)*IL + Vout*I_load), reescrita
 * como uma quadratica em IL: a*IL^2 + b*IL + I_load_eq = 0, com
 * a = sum_i( R_L_i / (VD_i+Vout_ref) ) e b = -sum_i( Vin / (VD_i+Vout_ref) ).
 * E o termo de FEEDFORWARD nao-linear somado ao PI de tensao em control.c;
 * nao existe equivalente no C-PI classico do artigo CBA2026 (que so usa o
 * erro de tensao, sem calcular uma corrente de equilibrio teorica).
 *
 * [REVISAR: item #5 do relatorio] "a" e proporcional a R_L (perdas), que e
 * pequeno por design (conversor de baixo custo de conducao): a formula
 * (-b-sqrt(disc))/(2a) fica NUMERICAMENTE MAL-CONDICIONADA quando a -> 0
 * (divisao por numero pequeno amplifica qualquer erro em b/disc). Isso e
 * mais critico nos primeiros ciclos apos ligar, quando R_L e VD ainda estao
 * no chute nominal (RL_nom = 1 mOhm) e nao convergiram pelo MRAC/Adam,
 * podendo gerar um IL_eq (e portanto IL_ref) absurdamente grande, saturando o
 * duty antes mesmo de o sistema estabilizar. Nao ha soft-start.
 * ----------------------------------------------------------------------- */
float compute_IL_eq(const float theta_sys[][2], const float theta_load[],
                    const float x_hat[], const float Vout_ref)
{
    float I_load_eq = theta_load[IDX_G] * Vout_ref + theta_load[IDX_IDC];

    float a = 0.0f;
    float b = 0.0f;
    for (int i = 0; i < PHASES_COUNT; i++)
    {
        float denom = theta_sys[i][IDX_VD] + Vout_ref;
        a += theta_sys[i][IDX_RL] / denom; // [REVISAR] fica pequeno quando R_L (identificado) e pequeno -> mal-condicionado
        b -= x_hat[SIG_V_IN] / denom;
    }

    // Duas tentativas de discriminante: primeiro contra a carga de EQUILIBRIO estimada (disc_eq); se essa nao
    // tiver raiz real (disc_eq <= 0), tenta contra a carga MEDIDA/estimada agora (disc_meas) como alternativa.
    float disc_eq = b * b - 4.0f * a * I_load_eq;
    float disc_meas = b * b - 4.0f * a * x_hat[SIG_I_OUT];

    if (disc_eq > 0.0f)
    {
        return (-b - sqrtf(disc_eq)) / (2.0f * a); // [REVISAR] sensivel a erro quando "a" e pequeno (ver acima)
    }
    else if (disc_meas > 0.0f)
    {
        return (-b - sqrtf(disc_meas)) / (2.0f * a); // [REVISAR] mesma ressalva
    }

    return 0.0f; // nenhuma raiz real em nenhuma das duas tentativas: retorna referencia nula (fallback seguro)
}
