/* =============================================================================
 * adc.h  (ANOTADO): interface de leitura das 6 (ou 5) grandezas medidas.
 * Ver adc.c para a explicacao da calibracao bi-linear.
 * ============================================================================= */

#ifndef ADC_H
#define ADC_H

void adc_read_states(float z[], int count);

#endif
