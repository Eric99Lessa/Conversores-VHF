/* =============================================================================
 * config.h  (ANOTADO)
 * =============================================================================
 * Arquivo central de configuracao do firmware. Define:
 *   - qual dos dois papeis o binario vai assumir quando compilado (ESP de
 *     controle ou ESP de comunicacao), via COMMS_MODE;
 *   - parametros fisicos nominais da planta (C, L, R_L, V_D) usados como
 *     ponto de partida do identificador online em control.c;
 *   - mapeamento de pinos GPIO/PWM/ADC/SPI;
 *   - os coeficientes lineares de conversao ADC -> grandeza fisica;
 *   - a struct task_params_t, que e a "caixa de correio" compartilhada entre
 *     a task de controle (produtora) e a task SPI escrava (consumidora) no
 *     ESP de controle, e tambem o layout usado pelo ESP de comunicacao para
 *     montar comandos.
 *
 * Nao ha logica de controle aqui, apenas constantes e tipos. Revisado em
 * 23/09/2026 junto com control.c, model.c, observer.c, optim.c,
 * communication.c e uart_task.c (ver relatorio
 * pos_doc/report/Diagnostico_Firmware_Conversores_VHF.pdf).
 * ============================================================================= */

#ifndef CONFIG_H
#define CONFIG_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/mcpwm.h" // Driver LEGADO do ESP-IDF (nao o novo driver "mcpwm" baseado em RMT/MCPWM v2).
                           // [CRITICAL] mcpwm_set_duty() deste driver espera duty em PORCENTAGEM (0.0-100.0),
                           // nao em fracao (0.0-1.0). Ver control.c, funcao duty_cascade() e update_duty().

typedef enum
{
    COMMS_MODE_OFF = 0, // Este binario roda no ESP "DE CONTROLE": laco de controle (control_task) + escravo SPI.
    COMMS_MODE_ON       // Este binario roda no ESP "DE COMUNICACAO": mestre SPI + console UART (uart_task).
} comms_mode_t;

typedef enum
{
    OP_MODE_STANDALONE = 0, // Conversor opera sozinho: a propria malha de controle calcula IL_ref a partir de Vout_ref
                            // (equilibrio de potencia + PI de tensao). E o modo usado nos ensaios de bancada atuais.
    OP_MODE_INTEGRATED      // Conversor e um bloco de um sistema maior: IL_ref vem pronto de fora (via SPI), sem
                            // o calculo de equilibrio nem o PI de tensao interno.
} operation_mode_t;

// Selecao de qual firmware sera compilado. Trocar aqui e recompilar decide o papel do ESP32.
#define COMMS_MODE COMMS_MODE_OFF
#define OPERATION_MODE OP_MODE_STANDALONE

// ===================== PWM =====================
#define PWM_FREQUENCY_HZ 20000   // Frequencia de chaveamento de CADA fase (20 kHz). As 3 fases sao entrelacadas
                                  // (defasadas 120 graus), entao a ondulacao efetiva vista pelo barramento e a 60 kHz.
#define PWM_INIT_DUTY_PC 0.0f    // Duty inicial em PORCENTAGEM (0-100), usado so na inicializacao do driver MCPWM.
                                  // O sufixo "_PC" (percent) e a pista de que a API trabalha em porcentagem, nao fracao.
#define PWM_UNIT MCPWM_UNIT_0
#define PHASES_COUNT 3

// ===================== Parametros nominais da planta =====================
// Servem de "chute inicial" para o identificador online (MRAC + Adam, ver control.c). Sao corrigidos em tempo real
// para R_L e V_D; C e L permanecem fixos (nao sao identificados online).
#define C_nom 0.012f   // Capacitancia de saida nominal [F]
#define L_nom 0.08f    // Indutancia de fase nominal [H]. [REVISAR] 80 mH e um valor alto para um conversor
                       // entrelacado de 20 kHz: confirmar contra o indutor fisico/BOM (item #2 do relatorio).
#define RL_nom 0.001f  // Resistencia serie do indutor, chute inicial [Ohm]
#define VD_nom 0.8f    // Queda de tensao no diodo/retificacao sincrona, chute inicial [V]

// Faixas de saturacao (clamp) dos parametros identificados online: evita que o MRAC/Adam divirja para valores
// fisicamente impossiveis.
#define RL_MIN 0.001f
#define RL_MAX 1.0f
#define VD_MIN 0.7f
#define VD_MAX 2.0f

// Faixas de saturacao do modelo de carga identificado (I_out = G*Vout + Idc)
#define G_LOAD_MAX 2.0f
#define IDC_LOAD_MAX 150.0f

// ===================== Mapeamento de GPIO =====================
// Pinos PWM: "_L_" = gate do interruptor de BAIXO (bottom/synchronous), "_U_" = gate do interruptor DE CIMA (top).
#define U_L_GPIO (22)
#define V_L_GPIO (21)
#define W_L_GPIO (5)
#define U_U_GPIO (23)
#define V_U_GPIO (3)
#define W_U_GPIO (18)
// Pinos de FALTA (fault) vindos do driver de gate SiC (SKYPER 42 LJ): nivel baixo = falta detectada em hardware.
#define U_ERR_GPIO (1)
#define V_ERR_GPIO (19)
#define W_ERR_GPIO (17)

// ===================== Periodo de controle =====================
#define CONTROL_RATE_HZ 1000                              // Taxa de execucao do laco de controle (aquisicao +
                                                            // observador + modelo + identificacao + lei de controle).
                                                            // [CRITICAL] E 20x MAIS LENTA que PWM_FREQUENCY_HZ
                                                            // (1 kHz vs 20 kHz): o duty so e recalculado a cada 20
                                                            // periodos de chaveamento. Malhas de corrente em
                                                            // conversores CC-CC tipicamente precisam de banda passante
                                                            // de centenas de Hz a poucos kHz para rejeitar
                                                            // perturbacoes; com atualizacao a 1 kHz, o ganho Kp_i do
                                                            // laco de corrente (duty_cascade em control.c) fica
                                                            // limitado a uma banda muito menor que a usada no projeto
                                                            // classico do artigo CBA2026 (crossover de corrente
                                                            // ~2 kHz / 12566 rad/s, com fs=10 kHz). Ver secao de
                                                            // comparacao no relatorio.
#define CONTROL_PERIOD_TICKS pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ)

// ===================== ADC =====================
#define ADC_SAMPLES_PER_CH 4           // Numero de leituras medias por canal a cada ciclo (oversampling simples).
#define ADC_WIDTH_CFG ADC_WIDTH_BIT_12
#define ADC_ATTEN_CFG ADC_ATTEN_DB_11
#define ADC_MEAS_IL_OUT 0              // 0: I_out e ESTIMADO pelo modelo (model_estimate_iload), nao medido por ADC.
                                        // 1: I_out e medido por um canal ADC dedicado. Ver Z_COUNT abaixo.
#define ADC_THR 2890                   // Limiar (em contagens ADC) que seleciona qual dos dois segmentos lineares de
                                        // calibracao usar em adc_read2meas() (adc.c): calibracao bi-linear por faixa.

// ===================== Pinos SPI =====================
#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK 14
#define PIN_CS 27

// ===================== Cores/prioridades das tasks FreeRTOS =====================
#define CORE_MAIN 0
#define CORE_SCND 1
#define PRIO_LOW 0
#define PRIO_HIGH 1

// Modo de operacao do conversor: boost (eleva Vout > Vin) ou buck (abaixa Vout < Vin).
// Definido por hardware/topologia de uso; alterado em runtime via comando SPI/UART "P".
typedef enum
{
    PWM_MODE_BOOST = 0,
    PWM_MODE_BUCK // 1
} pwm_mode_t;

// Lei de controle ativa. Alterada em runtime via comando SPI/UART "M".
typedef enum
{
    CTRL_OPEN = 0,   // Duty manual (malha aberta), recebido via comando SPI CMD_DUTY.
    CTRL_CASCADE,    // Cascata corrente(interna)/tensao(externa) por linearizacao por realimentacao. Ver control.c.
    CTRL_IDA_PBC     // [CRITICAL] Selecionavel via comando "M idapbc", mas NAO IMPLEMENTADO em control_task()
                     // (cai no "default" do switch, que apenas mantem o ultimo duty calculado: ver control.c).
} control_mode_t;

// Indices dos 6 sinais medidos/estimados pelo observador. A ORDEM importa: e usada para indexar arrays x[],
// x_hat[], z[], R[], Q_val[], Q_dot[] e os coeficientes ADC_*_COEFF* abaixo.
typedef enum
{
    SIG_IL1 = 0,
    SIG_IL2,
    SIG_IL3,
    SIG_V_OUT,
    SIG_V_IN,
    SIG_I_OUT,
    SIG_COUNT
} adc_signal_t;

// Indices dos parametros de planta identificados online, por fase: R_L (serie do indutor) e V_D (queda de diodo).
typedef enum
{
    IDX_RL = 0,
    IDX_VD,
    PARAMS_SYS_COUNT
} params_sys_idx_t;

// Indices dos parametros do modelo de carga identificado online: I_out = G*Vout + Idc.
typedef enum
{
    IDX_G = 0,
    IDX_IDC,
    PARAMS_LOAD_COUNT
} params_load_idx_t;

// Indices do vetor de balanco de potencia calculado por model_power() (usado so para telemetria/diagnostico).
typedef enum
{
    IDX_P_IN = 0,
    IDX_P_OUT,
    IDX_P_R,
    IDX_P_D,
    IDX_P_L,
    IDX_P_C,
    P_COUNT
} power_idx_t;

// Quantidade de sinais efetivamente medidos por ADC (SIG_COUNT - 1 se I_out for estimado, SIG_COUNT se for medido).
#define Z_COUNT (SIG_COUNT - 1 + ADC_MEAS_IL_OUT)

extern const adc_channel_t adc_sig_channel[SIG_COUNT]; // Definido em adc.c: mapeia cada SIG_* a um canal ADC1.

// Coeficientes de calibracao ADC -> grandeza fisica (contagem*a + b), em DOIS segmentos lineares (1: abaixo de
// ADC_THR, 2: acima), usados em adc_read2meas() (adc.c). Foram obtidos por ajuste de curva na bancada, nao por
// datasheet: nao ha comentario no repositorio original sobre a origem exata desses numeros.
#define ADC_LIN_COEFF1 {              \
    [SIG_IL1] = 0.016355301309278f,   \
    [SIG_IL2] = 0.016355301309278f,   \
    [SIG_IL3] = 0.016355301309278f,   \
    [SIG_V_OUT] = 0.046537663880674f, \
    [SIG_V_IN] = 0.031158767346203f,  \
    [SIG_I_OUT] = 0.016355301309278f, \
}

#define ADC_IND_COEFF1 {              \
    [SIG_IL1] = 3.080442675490790f,   \
    [SIG_IL2] = 3.080442675490790f,   \
    [SIG_IL3] = 3.080442675490790f,   \
    [SIG_V_OUT] = 6.700344761853321f, \
    [SIG_V_IN] = 5.047952876931346f,  \
    [SIG_I_OUT] = 3.080442675490790f, \
}

#define ADC_LIN_COEFF2 {              \
    [SIG_IL1] = 0.012291085427038f,   \
    [SIG_IL2] = 0.012291085427038f,   \
    [SIG_IL3] = 0.012291085427038f,   \
    [SIG_V_OUT] = 0.046537663880674f, \
    [SIG_V_IN] = 0.020762618848175f,  \
    [SIG_I_OUT] = 0.012291085427038f, \
}

#define ADC_IND_COEFF2 {               \
    [SIG_IL1] = 14.826026575166821f,   \
    [SIG_IL2] = 14.826026575166821f,   \
    [SIG_IL3] = 14.826026575166821f,   \
    [SIG_V_OUT] = 6.700344761853321f,  \
    [SIG_V_IN] = 35.092822036233684f,  \
    [SIG_I_OUT] = 14.826026575166821f, \
}

// Variancia de ruido de MEDICAO (R) usada pelo filtro de Kalman escalar (observer.c): mesmo valor para todos os
// sinais, o que e uma simplificacao (na pratica, tensao e corrente tem ruidos de medicao com escalas diferentes).
#define R_MEAS {         \
    [SIG_IL1] = 0.01f,   \
    [SIG_IL2] = 0.01f,   \
    [SIG_IL3] = 0.01f,   \
    [SIG_V_OUT] = 0.01f, \
    [SIG_V_IN] = 0.01f,  \
    [SIG_I_OUT] = 0.01f, \
}

// Variancia de ruido de PROCESSO do filtro de Kalman, separada para o valor (Q_VAL) e para a derivada (Q_DOT) de
// cada estado. Q_DOT >> Q_VAL indica que o filtro confia pouco no modelo de derivada constante entre amostras
// (razoavel, ja que a derivada real muda a cada ciclo de chaveamento).
#define Q_VAL {            \
    [SIG_IL1] = 0.0001f,   \
    [SIG_IL2] = 0.0001f,   \
    [SIG_IL3] = 0.0001f,   \
    [SIG_V_OUT] = 0.0001f, \
    [SIG_V_IN] = 0.0001f,  \
    [SIG_I_OUT] = 0.0001f, \
}

#define Q_DOT {         \
    [SIG_IL1] = 1.0f,   \
    [SIG_IL2] = 1.0f,   \
    [SIG_IL3] = 1.0f,   \
    [SIG_V_OUT] = 1.0f, \
    [SIG_V_IN] = 1.0f,  \
    [SIG_I_OUT] = 1.0f, \
}

// Struct compartilhada entre a task de controle (escreve x, x_dot, theta_*, d_bot, d_top) e a task SPI escrava
// (le tudo para telemetria, escreve pwm_mode/ctrl_mode/Vout_ref/IL_ref/d_cmd vindos de comandos remotos).
// Protegida por portMUX (telem_mux) nas secoes criticas dentro de control.c e communication.c.
typedef struct
{
    portMUX_TYPE telem_mux;
    volatile uint8_t pwm_mode;  // e.g. PWM_MODE_BOOST
    volatile uint8_t ctrl_mode; // atualizado pela task SPI em tempo de execucao
    volatile float z[Z_COUNT];
    volatile float x[SIG_COUNT];
    volatile float x_pred[SIG_COUNT];
    volatile float x_dot[SIG_COUNT];
    volatile float x_dot_model[PHASES_COUNT + 1];
    volatile float theta_sys[PHASES_COUNT][PARAMS_SYS_COUNT];
    volatile float theta_load[PARAMS_LOAD_COUNT];
    volatile float d_bot[PHASES_COUNT];
    volatile float d_top[PHASES_COUNT];
    volatile float Vout_ref;
    volatile float IL_ref;
    volatile float d_cmd[PHASES_COUNT];

} task_params_t;

#endif
