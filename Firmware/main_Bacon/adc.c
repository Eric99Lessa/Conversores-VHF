/* =============================================================================
 * adc.c  (ANOTADO)
 * =============================================================================
 * Le e converte as 6 grandezas fisicas do conversor (3 correntes de fase,
 * Vout, Vin, Iout) a partir dos canais ADC1 do ESP32, usando uma calibracao
 * BI-LINEAR (2 segmentos: um para leituras abaixo de ADC_THR, outro acima),
 * com media de ADC_SAMPLES_PER_CH leituras por canal a cada chamada
 * (oversampling simples para reduzir ruido).
 * ============================================================================= */

#include "adc.h"
#include "config.h"

// Mapeamento de cada sinal logico (adc_signal_t, config.h) para o canal fisico ADC1 correspondente.
const adc_channel_t adc_sig_channel[SIG_COUNT] = {
    [SIG_V_IN] = ADC1_CHANNEL_0,
    [SIG_V_OUT] = ADC1_CHANNEL_3,
    [SIG_IL1] = ADC1_CHANNEL_7,
    [SIG_IL2] = ADC1_CHANNEL_4,
    [SIG_IL3] = ADC1_CHANNEL_5,
    [SIG_I_OUT] = ADC1_CHANNEL_6,
};

// Coeficientes de calibracao (contagem*a + b), dois segmentos por sinal: definidos em config.h.
static const float adc_lin_coeff1[SIG_COUNT] = ADC_LIN_COEFF1;
static const float adc_ind_coeff1[SIG_COUNT] = ADC_IND_COEFF1;
static const float adc_lin_coeff2[SIG_COUNT] = ADC_LIN_COEFF2;
static const float adc_ind_coeff2[SIG_COUNT] = ADC_IND_COEFF2;

// Media de "samples" leituras cruas (raw) do canal ch: oversampling simples para reduzir ruido de quantizacao.
static inline float adc_read_avg(adc1_channel_t ch, int samples)
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < samples; i++)
    {
        adc_reading += adc1_get_raw(ch);
    }
    return (float)adc_reading / samples;
}

/* adc_read2meas(): converte uma leitura crua (adc_value) em grandeza fisica usando uma reta y=a*x+b, escolhendo
 * o PAR (a,b) conforme adc_value estar abaixo ou acima do limiar "threshold" (ADC_THR): calibracao bi-linear por
 * faixa, tipicamente usada quando o sensor/condicionamento de sinal tem ganho diferente perto de zero (offset de
 * amplificadores, retificacao, etc.). Leitura exatamente 0 e tratada como caso especial (retorna 0 direto). */
static float adc_read2meas(float adc_value, float a_1, float b_1, float a_2, float b_2, int threshold)
{
    if (adc_value == 0)
    {
        return 0.0f;
    }
    if (adc_value < threshold)
    {
        return a_1 * adc_value + b_1;
    }
    else
    {
        return a_2 * adc_value + b_2;
    }
}

// Le e converte as "count" primeiras grandezas de adc_sig_channel[] (chamada de control.c com count = Z_COUNT).
void adc_read_states(float z[], int count)
{
    for (int i = 0; i < count; i++)
    {
        float raw = adc_read_avg(adc_sig_channel[i], ADC_SAMPLES_PER_CH);
        z[i] = adc_read2meas(raw, adc_lin_coeff1[i], adc_ind_coeff1[i], adc_lin_coeff2[i], adc_ind_coeff2[i], ADC_THR);
    }
}
