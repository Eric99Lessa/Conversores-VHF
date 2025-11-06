//                          FIRMWARE BOOST INTERLEAVED

//===================================================================================================
//                          CABECALHOS
//===================================================================================================

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"

//===================================================================================================
//                          DEFINIÇÕES DO SISTEMA
//===================================================================================================

#define LEDC_FREQUENCY (14000)                //------------- Frequency in Hz
#define LEDC_HS_TIMER LEDC_TIMER_0            //-------- Timer do PWM
#define LEDC_HS_MODE LEDC_LOW_SPEED_MODE      //- Modo do PWM
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0    //------ Canal do PWM da fase 1
#define LEDC_HS_CH1_CHANNEL LEDC_CHANNEL_1    //------ Canal do PWM da fase 2
#define LEDC_HS_CH2_CHANNEL LEDC_CHANNEL_2    //------ Canal do PWM da fase 3
#define PWM_FASE_1 (5)                        //----------------- Pino do PWM da fase 1
#define PWM_FASE_2 (18)                       //---------------- Pino do PWM da fase 2
#define PWM_FASE_3 (19)                       //---------------- Pino do PWM da fase 3
#define CANAL_CORRENTE_L1 ADC_CHANNEL_0       //------- PINO 36
#define CANAL_CORRENTE_L2 ADC_CHANNEL_3       //------- PINO 34
#define CANAL_CORRENTE_L3 ADC_CHANNEL_6       //------- PINO 39
#define CANAL_TENSAO_ENTRADA ADC_CHANNEL_7    //------- PINO 35
#define DEFAULT_VREF 1100                     //---------------- Valor de referencia do ADC
#define QUANTIDADE_BITS_PWM LEDC_TIMER_10_BIT //--- Quantidade de bits do PWM
#define NO_OF_SAMPLES 64                      //---------------- numero de leituras do ADC
#define BUF_SIZE (1024)
#define K_tensao 23.589165980615250
#define K_corrente 19.220283160866238 // Multimetro
// #define K_corrente 19.851648319880304 // Osciloscopio

float interpola_dois_pontos(float x, float x0, float x1, float y0, float y1);
//===================================================================================================
//                         VARIAVEIS GLOBAIS DO SISTEMA
//===================================================================================================
esp_adc_cal_characteristics_t *adc_chars; //-- VARIAVEL DE CALIBRAÇÃO DE ADC
uint8_t dutyGlobal = 0;                   //-------------------- Duty do de controle do PWM

//===================================================================================================
//                              init_uart (void *arg)
//
// Proposito da função: Iniciliar o periférico da UART
//
// Parametros de entrada: NENHUM
// Parametros de Saída  : NENHUM
//===================================================================================================
void init_uart(void)
{
    // configura UART1
    uart_config_t uart1_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    // define pinos de comunicação e instala drivers
    uart_param_config(UART_NUM_0, &uart1_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
}

//===================================================================================================
//                              taskLeituraUart (void *arg)
//
// Proposito da Tarefa: Tem como proposito receber os valores da UART e interpretar os dados
//                      para realizar os comandos de duty do PWM
//
// Parametros de entrada: NENHUM
// Parametros de Saída  : NENHUM
//===================================================================================================
void taskLeituraUart(void *arg)
{
    uart_port_t uart_num = UART_NUM_0; // Numero da porta da UART
    uint8_t data[BUF_SIZE];            // Variavel que receberá os dados da uart
    while (1)
    {
        int bytesRead = uart_read_bytes(uart_num, data, BUF_SIZE, 1000 / portTICK_PERIOD_MS); // Aguarda por 1 segundo a mensagem
        if (bytesRead > 0)
        {
            if (data[0] == 'A')
            { // Se receber 'A' aumenta 1% do duty
                dutyGlobal = dutyGlobal + 1;
                printf("\nDUTY = %i \n", dutyGlobal);
            } //////////////////////////////////////////////////////////
            else if (data[0] == 'B')
            { // Se receber 'B' diminui 1% do duty
                if (dutyGlobal > 0)
                    dutyGlobal = dutyGlobal - 1;
                printf("\nDUTY = %i \n", dutyGlobal);
            } //////////////////////////////////////////////////////////
            if (data[0] == 'C')
            { // Se receber 'C' aumenta 5% do duty
                dutyGlobal = dutyGlobal + 5;
                printf("\nDUTY = %i \n", dutyGlobal);
            } //////////////////////////////////////////////////////////
            else if (data[0] == 'D')
            { // Se receber 'D' diminui 5% do duty
                if (dutyGlobal > 5)
                    dutyGlobal = dutyGlobal - 5;
                printf("\nDUTY = %i \n", dutyGlobal);
            } //////////////////////////////////////////////////////////
            if (data[0] == 'E')
            { // Se receber 'E' zera o duty
                dutyGlobal = 0;
                printf("\nDUTY = %i \n", dutyGlobal);
            } //////////////////////////////////////////////////////////
        }
    }
}

//===================================================================================================
//                              binaryToVoltage (float adc_value)
//
// Proposito da Tarefa: Recebe a leitura dos bits do conversor analogico/digital e converte para uma
// tensao em V
//
// Parametros de entrada: adc_value (binario)
// Parametros de Saída  : adc_voltage
//===================================================================================================

float binaryToVoltage(float adc_value)
{
    if (adc_value == 0)
    {
        return 0;
    }
    if (adc_value > 3028)
    {
        return 0.000486251139392 * adc_value + 1.149508344943664;
    }
    return 0.000818852818358 * adc_value + 0.142390461037213;
}

//===================================================================================================
//                              taskLeituraADC (void *arg)
//
// Proposito da Tarefa: Tem como proposito realizar a leitura dos ADC
//
//
// Parametros de entrada: NENHUM
// Parametros de Saída  : NENHUM
//===================================================================================================
void taskLeituraADC(void *arg)
{
    uint32_t adc_reading = 0; // valor da leitura dos ADCs
    float CorrenteL1 = 0;     // valor da corrente do indutor 1 [A]
    float CorrenteL2 = 0;     // valor da corrente do indutor 2 [A]
    float CorrenteL3 = 0;     // valor da corrente do indutor 3 [A]
    float TensaoEntrada = 0;  // valor da tensao de entrada [V]
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(CANAL_CORRENTE_L1, ADC_ATTEN_DB_11);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    while (true)
    {
        /////////////////////////////////////////////////////////////////////////
        adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)CANAL_CORRENTE_L1);
        }
        adc_reading /= NO_OF_SAMPLES;
        CorrenteL1 = K_corrente * binaryToVoltage(adc_reading);
        /////////////////////////////////////////////////////////////////////////
        adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)CANAL_CORRENTE_L2);
        }
        adc_reading /= NO_OF_SAMPLES;
        CorrenteL2 = K_corrente * binaryToVoltage(adc_reading);
        /////////////////////////////////////////////////////////////////////////
        adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)CANAL_CORRENTE_L3);
        }
        adc_reading /= NO_OF_SAMPLES;
        CorrenteL3 = K_corrente * binaryToVoltage(adc_reading);
        /////////////////////////////////////////////////////////////////////////
        adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)CANAL_TENSAO_ENTRADA);
        }
        adc_reading /= NO_OF_SAMPLES;
        TensaoEntrada = K_tensao * binaryToVoltage(adc_reading);

        printf("Duty Cycle: %i, Corrente L1: %f, Corrente L2: %f, Corrente L3: %f, Tensao Entrada: %f\n", dutyGlobal, CorrenteL1, CorrenteL2, CorrenteL3, TensaoEntrada);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//===================================================================================================
//                interpola_dois_pontos(float x, float x0, float x1, float y0, float y1)
//
// Proposito da Tarefa: Função de interpolação de dois ponto
//
// Parametros de entrada: float x - valor atual
//                        float x0 - valor do x antes do x atual
//                        float x1 - valor do x depois do x atual
//                        float y0 - valor do y do x0
//                        float y1 - valor do y do x1
// Parametros de Saída  : float (valor y interpolado)
//===================================================================================================
float interpola_dois_pontos(float x, float x0, float x1, float y0, float y1)
{
    if (x1 == x0)
        return y1;
    else
        return (y0 + (x - x0) * (y1 - y0) / (x1 - x0));
}

void ControlePWM(void *arg)
{
    // Configuração do Timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = QUANTIDADE_BITS_PWM, // Resolução do PWM
        .freq_hz = LEDC_FREQUENCY,              // frequencia do PWM
        .speed_mode = LEDC_HS_MODE,             // Timer mode
        .timer_num = LEDC_HS_TIMER,             // Timer index
        .clk_cfg = LEDC_AUTO_CLK                // Selecione automaticamente o Timer de origem
    };
    ledc_timer_config(&ledc_timer);

    // Prepara e aplica a configuração do canal LEDC
    ledc_channel_config_t ledc_channel[3] = {
        {.channel = LEDC_HS_CH0_CHANNEL, // Especifica o número do canal LEDC.
         .duty = 0,                      //--------------------- Define o ciclo de duty cicle para o sinal PWM
         .gpio_num = PWM_FASE_1,         //-------- Atribui o número do pino GPIO que vai emitir o sinal PWM
         .speed_mode = LEDC_HS_MODE,     //---- Especifica o modo de velocidade para o canal LEDC
         .hpoint = 0,                    //------------------- Define o valor do ponto alto, que determina o ponto no ciclo PWM onde o ciclo de trabalho começa (DEFASAGEM)
         .timer_sel = LEDC_HS_TIMER},    //--- Seleciona o temporizador a ser usado com o canal
        {.channel = LEDC_HS_CH1_CHANNEL, // Especifica o número do canal LEDC.
         .duty = 0,                      //--------------------- Define o ciclo de duty cicle para o sinal PWM
         .gpio_num = PWM_FASE_2,         //-------- Atribui o número do pino GPIO que vai emitir o sinal PWM
         .speed_mode = LEDC_HS_MODE,     //---- Especifica o modo de velocidade para o canal LEDC
         .hpoint = 341,                  //----------------- Define o valor do ponto alto, que determina o ponto no ciclo PWM onde o ciclo de trabalho começa (DEFASAGEM)
         .timer_sel = LEDC_HS_TIMER},    //--- Seleciona o temporizador a ser usado com o canal
        {.channel = LEDC_HS_CH2_CHANNEL, // Especifica o número do canal LEDC.
         .duty = 0,                      //--------------------- Define o ciclo de duty cicle para o sinal PWM
         .gpio_num = PWM_FASE_3,         //--------- Atribui o número do pino GPIO que vai emitir o sinal PWM
         .speed_mode = LEDC_HS_MODE,     //---- Especifica o modo de velocidade para o canal LEDC
         .hpoint = 682,                  //----------------- Define o valor do ponto alto, que determina o ponto no ciclo PWM onde o ciclo de trabalho começa (DEFASAGEM)
         .timer_sel = LEDC_HS_TIMER},
    }; //-- Seleciona o temporizador a ser usado com o canal

    // Define o duty cicle para os canais
    for (int ch = 0; ch < 3; ch++)
    {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Atualiza o valor do duty cicle dos PWMs
    ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
    ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
    ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, 0);
    ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
    ledc_set_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel, 0);
    ledc_update_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel);

    vTaskDelay(pdMS_TO_TICKS(3000)); //------------ delay de 3 ms
    uint16_t dutyDesejado = 0;
    while (true)
    {
        dutyDesejado = (uint16_t)interpola_dois_pontos((float)dutyGlobal, (float)0, (float)100, (float)0, (float)1024);
        ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, dutyDesejado);
        ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
        ledc_set_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel, dutyDesejado);
        ledc_update_duty(ledc_channel[1].speed_mode, ledc_channel[1].channel);
        ledc_set_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel, dutyDesejado);
        ledc_update_duty(ledc_channel[2].speed_mode, ledc_channel[2].channel);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Roda a tarefa a cada 1ms
    } // fim do while (true)
} // fim do ControlePWM(void *arg)

//===================================================================================================
//                                  void app_main(void)
//
// Proposito da Tarefa: Função principal do sistema, todas as tarefas devem ser criadas aqui
//
// Parametros de entrada: NENHUM
// Parametros de Saída  : NENHUM
//===================================================================================================
void app_main(void)
{
    init_uart();                                                                         // inicializa a uart
    xTaskCreatePinnedToCore(ControlePWM, "ControlePWM", 4096, NULL, 2, NULL, 1);         // cria a tarefa do controle de PWM
    xTaskCreatePinnedToCore(taskLeituraADC, "LeituraADC", 4096, NULL, 2, NULL, 1);       // cria a tarefa de leitura do ADC
    xTaskCreatePinnedToCore(taskLeituraUart, "taskLeituraUart", 4096, NULL, 2, NULL, 1); // cria a tarefa de leitura da uart
}