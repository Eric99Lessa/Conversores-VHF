#include "adc.h"
#include "config.h"

const adc_channel_t adc_sig_channel[SIG_COUNT] = {
    [SIG_V_IN] = ADC1_CHANNEL_0,
    [SIG_V_OUT] = ADC1_CHANNEL_3,
    [SIG_IL1] = ADC1_CHANNEL_7,
    [SIG_IL2] = ADC1_CHANNEL_4,
    [SIG_IL3] = ADC1_CHANNEL_5,
    [SIG_I_OUT] = ADC1_CHANNEL_6,
};

static const float adc_lin_coeff1[SIG_COUNT] = ADC_LIN_COEFF1;
static const float adc_ind_coeff1[SIG_COUNT] = ADC_IND_COEFF1;
static const float adc_lin_coeff2[SIG_COUNT] = ADC_LIN_COEFF2;
static const float adc_ind_coeff2[SIG_COUNT] = ADC_IND_COEFF2;

static inline float adc_read_avg(adc1_channel_t ch, int samples)
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < samples; i++)
    {
        adc_reading += adc1_get_raw(ch);
    }
    return (float)adc_reading / samples;
}

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

void adc_read_states(float z[], int count)
{
    for (int i = 0; i < count; i++)
    {
        float raw = adc_read_avg(adc_sig_channel[i], ADC_SAMPLES_PER_CH);
        z[i] = adc_read2meas(raw, adc_lin_coeff1[i], adc_ind_coeff1[i], adc_lin_coeff2[i], adc_ind_coeff2[i], ADC_THR);
    }
}