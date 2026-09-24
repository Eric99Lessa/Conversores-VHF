/* =============================================================================
 * control.c  (ANOTADO: revisao de 23/09/2026)
 * =============================================================================
 * Coracao do firmware do ESP "de controle". Implementa, a CONTROL_RATE_HZ
 * (1 kHz, config.h):
 *   1) leitura das 6 grandezas via ADC (ou estimativa de I_out);
 *   2) filtragem por Kalman escalar (observer.c) -> x_hat, x_dot_hat;
 *   3) predicao das derivadas pelo modelo medio do conversor (model.c);
 *   4) identificacao online (MRAC + Adam, optim.c) de R_L, V_D (por fase) e
 *      do modelo de carga G, Idc;
 *   5) calculo da referencia de corrente IL_ref (modo STANDALONE): equilibrio
 *      de potencia (compute_IL_eq, model.c) + PI de tensao;
 *   6) lei de controle (CTRL_OPEN / CTRL_CASCADE / CTRL_IDA_PBC) -> duty[];
 *   7) atuacao no hardware MCPWM;
 *   8) publicacao da telemetria para a task SPI.
 *
 * COMPARACAO COM O ARTIGO CBA2026 (PI cascata classico, papera_r3.tex):
 *   O artigo usa DOIS PIs LINEARES fixos, projetados por resposta em
 *   frequencia (cruzamento + margem de fase) em tempo continuo:
 *     - Tensao (externa): Kpv=0.0576, Kiv=0.6331, wc=2*pi*20 rad/s, MF=85 grs
 *     - Corrente (interna): Kpc=1.5003e-4, Kic=0.1922, wc=2*pi*2000 rad/s, MF=85 grs
 *   Aqui neste firmware:
 *     - O laco EXTERNO (tensao) TEM a mesma estrutura de um PI (Kp_v, Ki_v
 *       sobre err_V), mas SOMA um termo extra de feedforward nao-linear
 *       (IL_eq, de compute_IL_eq) que o C-PI do artigo NAO tem.
 *     - O laco INTERNO (corrente) NAO E um PI. duty_cascade() resolve
 *       ALGEBRICAMENTE a EDO continua da corrente de indutor para o duty D
 *       que impoe a dinamica de erro de 1a ordem escolhida (linearizacao por
 *       realimentacao / model inversion), usando os parametros R_L, V_D
 *       identificados ONLINE. So existe Kp_i (ganho proporcional / posicao
 *       do polo de malha fechada em rad/s); NAO HA termo integral na malha
 *       de corrente, ao contrario do Kic do artigo.
 *   CONCLUSAO: as duas leis NAO SAO EQUIVALENTES. O laco de tensao e
 *   estruturalmente parecido (PI) mas com feedforward adicional; o laco de
 *   corrente e um controlador nao-linear adaptativo por inversao de modelo,
 *   nao um PI classico. Isso e esperado por design (o firmware tenta ser
 *   mais "avancado" que o C-PI do artigo), mas significa que os ganhos
 *   Kpc/Kic do artigo NAO PODEM ser copiados diretamente para Kp_i aqui:
 *   as unidades e o papel de cada ganho sao diferentes.
 *
 * REPRESENTACAO EM TEMPO DISCRETO DOS GANHOS:
 *   - Ki_v (integrador de tensao): a integracao "int_eV += Ts*err_V" e uma
 *     discretizacao correta por Euler-avante do integrador continuo 1/s.
 *     Estruturalmente certo; o VALOR de Ki_v (1.0, placeholder) e que nao
 *     foi projetado (ver item [CRITICAL] abaixo).
 *   - Kp_v: termo proporcional, nao precisa de escala por Ts. OK.
 *   - Kp_i (laco de corrente): NAO tem nenhuma dependencia de Ts no codigo;
 *     e usado diretamente como se a lei fosse continua, apenas AVALIADA a
 *     cada amostra (controle "quase-continuo"). Isso e uma pratica valida,
 *     MAS exige que o polo de malha fechada fique bem abaixo da frequencia
 *     de amostragem do LACO DE CONTROLE (CONTROL_RATE_HZ = 1 kHz), nao da
 *     PWM (20 kHz). Regra pratica: banda de malha fechada <= ~1/10 a 1/5 da
 *     taxa de amostragem do laco que a implementa, ou seja Kp_i deveria
 *     ficar abaixo de ~100-200 rad/s para ser fielmente representado a
 *     1 kHz. O artigo, em comparacao, projeta o laco de corrente para
 *     ~12566 rad/s (2 kHz) rodando a fs=10 kHz: ja um caso limite. Rodar
 *     uma malha de corrente de conversor a apenas 1 kHz de atualizacao
 *     (20x mais lento que o PWM) e uma limitacao arquitetural que deve ser
 *     discutida com o Eric independentemente do bug de unidade do duty.
 * ============================================================================= */

#include "control.h"
#include "config.h"
#include "adc.h"
#include "observer.h"
#include "model.h"
#include "optim.h"
#include "math.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "string.h"

_Static_assert(ADC_MEAS_IL_OUT == 0 || ADC_MEAS_IL_OUT == 1,
               "ADC_MEAS_IL_OUT must be 0 or 1");

/* -----------------------------------------------------------------------
 * mcpwm_setup(): inicializa os 3 timers MCPWM (um por fase), o entrelacamento
 * de 120 graus entre eles via sincronismo (sync_cfg.timer_val = 0/333/667 de
 * um periodo normalizado de 1000), e a protecao de hardware por falta
 * (F0/F1/F2 ligados aos pinos *_ERR_GPIO do driver de gate SiC): qualquer
 * falta forca os dois braços (A e B) de todos os timers para nivel baixo,
 * desligando os IGBTs/MOSFETs independentemente do que o firmware calcular.
 * ----------------------------------------------------------------------- */
static esp_err_t mcpwm_setup(void)
{
    // GPIO INIT PWM: associa cada sinal logico MCPWMxA/B a um pino fisico.
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM0A, U_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM0B, U_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM1A, V_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM1B, V_U_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM2A, W_L_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM2B, W_U_GPIO));

    // GPIO INIT FAULT: entradas digitais dos sinais de falta do driver de gate.
    gpio_config_t fault_gpio_conf = {
        .pin_bit_mask = (1ULL << U_ERR_GPIO) | (1ULL << V_ERR_GPIO) | (1ULL << W_ERR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&fault_gpio_conf));

    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM_FAULT_0, U_ERR_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM_FAULT_1, V_ERR_GPIO));
    ESP_ERROR_CHECK(mcpwm_gpio_init(PWM_UNIT, MCPWM_FAULT_2, W_ERR_GPIO));

    // Frequencia e valor inicial (em PORCENTAGEM, cmpr_a/cmpr_b) de cada timer.
    mcpwm_config_t pwm_cfg = {
        .frequency = PWM_FREQUENCY_HZ,
        .cmpr_a = PWM_INIT_DUTY_PC,
        .cmpr_b = PWM_INIT_DUTY_PC,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, MCPWM_TIMER_0, &pwm_cfg));
    ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, MCPWM_TIMER_1, &pwm_cfg));
    ESP_ERROR_CHECK(mcpwm_init(PWM_UNIT, MCPWM_TIMER_2, &pwm_cfg));

    // Sincronismo: timer 0 e a referencia (sync_sig gerado no zero do contador, TEZ); timers 1 e 2 sao
    // realinhados para comecar em 333/667 de um periodo normalizado de 1000, ou seja, defasados 120 e 240 graus.
    mcpwm_set_timer_sync_output(PWM_UNIT, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);

    mcpwm_sync_config_t sync_cfg = {
        .sync_sig = MCPWM_SELECT_TIMER0_SYNC,
        .timer_val = 0,
        .count_direction = MCPWM_TIMER_COUNT_MODE_UP,
    };

    ESP_ERROR_CHECK(mcpwm_sync_configure(PWM_UNIT, MCPWM_TIMER_0, &sync_cfg));
    sync_cfg.timer_val = 333; // fase V a 120 graus
    ESP_ERROR_CHECK(mcpwm_sync_configure(PWM_UNIT, MCPWM_TIMER_1, &sync_cfg));
    sync_cfg.timer_val = 667; // fase W a 240 graus
    ESP_ERROR_CHECK(mcpwm_sync_configure(PWM_UNIT, MCPWM_TIMER_2, &sync_cfg));

    // Protecao por falta de hardware: nivel baixo em qualquer *_ERR_GPIO forca ambos os bracos (A e B) de
    // TODOS os timers para nivel baixo (force-low), desligando os interruptores em hardware.
    ESP_ERROR_CHECK(mcpwm_fault_init(PWM_UNIT, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F0));
    ESP_ERROR_CHECK(mcpwm_fault_init(PWM_UNIT, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F1));
    ESP_ERROR_CHECK(mcpwm_fault_init(PWM_UNIT, MCPWM_LOW_LEVEL_TGR, MCPWM_SELECT_F2));

    mcpwm_output_action_t action_a = MCPWM_ACTION_FORCE_LOW;
    mcpwm_output_action_t action_b = MCPWM_ACTION_FORCE_LOW;

    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(PWM_UNIT, MCPWM_TIMER_0, MCPWM_SELECT_F0, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(PWM_UNIT, MCPWM_TIMER_1, MCPWM_SELECT_F1, action_a, action_b));
    ESP_ERROR_CHECK(mcpwm_fault_set_cyc_mode(PWM_UNIT, MCPWM_TIMER_2, MCPWM_SELECT_F2, action_a, action_b));

    return ESP_OK;
}

/* -----------------------------------------------------------------------
 * update_duty(): aplica o array duty[] (uma entrada por fase) no operador
 * MCPWM correto (A para boost/braço de baixo, B para buck/braço de cima).
 *
 * [CRITICAL: item #1 do relatorio] mcpwm_set_duty() (driver LEGADO
 * driver/mcpwm.h) espera o argumento de duty em PORCENTAGEM (0.0 a 100.0),
 * exatamente como PWM_INIT_DUTY_PC e cmpr_a/cmpr_b sao inicializados acima.
 * Todo o resto do firmware (duty_cascade(), duty_open(), o comando SPI
 * CMD_DUTY em communication.c e o comando de console "D" em uart_task.c)
 * trabalha com duty como FRACAO em [0,1]. NAO HA, em nenhum lugar do
 * codigo, uma multiplicacao por 100 entre o calculo do duty (fracao) e
 * esta chamada. Resultado esperado: o duty efetivamente aplicado ao gate e
 * ~100x menor que o calculado (ex.: 0,3% em vez de 30%): isso e o
 * candidato mais forte para explicar por que CTRL_CASCADE nao funciona.
 *
 * CORRECAO SUGERIDA: multiplicar duty[i] por 100.0f logo abaixo, antes de
 * cada chamada de mcpwm_set_duty (ou fazer isso dentro desta funcao, que e
 * o unico ponto de contato com a HAL, deixando duty_open/duty_cascade e o
 * protocolo de comunicacao 100% em fracao [0,1], que e mais natural
 * matematicamente).
 * ----------------------------------------------------------------------- */
static void update_duty(const float duty[3], pwm_mode_t mode)
{
    if (mode == PWM_MODE_BOOST)
    {
        // [CRITICAL] duty[i] esta em fracao [0,1] aqui: mcpwm_set_duty espera [0,100].
        // Sugestao: mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty[0] * 100.0f);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_A, duty[0]);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_A, duty[1]);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_A, duty[2]);
    }
    else
    {
        // [CRITICAL] mesmo problema de unidade descrito acima, aqui para o modo buck (operador B).
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_0, MCPWM_OPR_B, duty[0]);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_1, MCPWM_OPR_B, duty[1]);
        mcpwm_set_duty(PWM_UNIT, MCPWM_TIMER_2, MCPWM_OPR_B, duty[2]);
    }
}

// ===== FUNCOES DE MODO DE CONTROLE =====

/* duty_open(): modo manual (CTRL_OPEN). Apenas repassa o duty recebido por
 * SPI (params->d_cmd, atualizado pelo comando CMD_DUTY) para a saida, sem
 * nenhuma realimentacao. Usado para caracterizar a planta em malha aberta. */
static void duty_open(float duty_out[3], float duty_in[3])
{
    duty_out[0] = duty_in[0];
    duty_out[1] = duty_in[1];
    duty_out[2] = duty_in[2];
}

/* -----------------------------------------------------------------------
 * duty_cascade(): lei de controle do modo CTRL_CASCADE. NAO e um PI: e a
 * SOLUCAO ALGEBRICA da equacao diferencial continua da corrente de indutor
 *     L * dIL/dt = Vin - IL*R_L - (Vout + VD) * (1 - D)      [modelo boost]
 * para o duty D que faz a corrente seguir a dinamica de erro de 1a ordem
 * escolhida:
 *     dIL/dt = -Kp_i * (IL - IL_ref)
 * Igualando as duas expressoes e isolando D chega-se exatamente na linha
 * abaixo (conferida nesta revisao como identica a
 * Matlab/Cascade_duty_Boost.m, gerada simbolicamente pelo MATLAB). Essa
 * tecnica se chama linearizacao por realimentacao (feedback linearization)
 * ou "controle por inversao de modelo": ao contrario de um PI, nao ha
 * estado de integrador na malha de corrente, e o resultado depende
 * diretamente dos parametros de planta R_L, V_D estimados ONLINE pelo
 * MRAC/Adam (theta_sys): ou seja, e um controlador ADAPTATIVO, nao um
 * ganho fixo linear como o PI classico do artigo CBA2026.
 *
 * [CRITICAL] duty_out[i] SEMPRE fica no intervalo [0,1] (fracao) por
 * construcao: ver aviso em update_duty() acima sobre a conversao de
 * escala faltante antes de mcpwm_set_duty().
 *
 * [REVISAR: item #3 do relatorio] esta funcao NAO recebe pwm_mode e usa
 * SEMPRE a equacao de regime do BOOST (o termo (Vin - x[i]*RL + ...) /
 * (VD+Vout) e especifico do boost). Comparar com model_derivatives()
 * abaixo, que ramifica explicitamente por PWM_MODE_BOOST/PWM_MODE_BUCK.
 * Enquanto pwm_mode == PWM_MODE_BOOST (o default em main.c) isso nao afeta
 * o resultado, mas testar CTRL_CASCADE em modo buck vai usar a lei errada
 * da planta.
 * ----------------------------------------------------------------------- */
static void duty_cascade(float duty_out[3], float x[SIG_COUNT], const float L[], float theta_sys[][PARAMS_SYS_COUNT], float IL_ref, float Kp_i)
{
    // 1 - (Vin-IL.*R_L+Kp_i.*L.*(IL-IL_ref))./(VD+Vout)

    // Le os estados de tensao (comuns as 3 fases)
    float Vin = x[SIG_V_IN];
    float Vout = x[SIG_V_OUT];

    for (int i = 0; i < PHASES_COUNT; i++)
    {
        // [CRITICAL] resultado em FRACAO [0,1]. [REVISAR] sempre equacao do boost (ver comentario acima da funcao).
        duty_out[i] = 1 - (Vin - x[i] * theta_sys[i][IDX_RL] + Kp_i * L[i] * (x[i] - IL_ref)) / (theta_sys[i][IDX_VD] + Vout);
    }
}

/* clamp_array(): satura cada elemento de theta[] entre theta_min[] e theta_max[]: usado para os parametros
 * identificados online (R_L, V_D, G, Idc), NAO para o duty (ver item [CRITICAL] #4 mais abaixo, no laco
 * principal: nao ha clamp equivalente para o array duty[] antes de update_duty()). */
void clamp_array(float theta[], const float theta_min[], const float theta_max[], int n)
{
    for (int j = 0; j < n; j++)
    {
        theta[j] = fmaxf(fminf(theta[j], theta_max[j]), theta_min[j]);
    }
}

static const char *TAG_PWM = "MCPWM";

/* =============================================================================
 * control_task(): task FreeRTOS principal do ESP de controle. Roda para
 * sempre (while(1)), fixada no core 0, ritmada por vTaskDelayUntil a
 * CONTROL_RATE_HZ = 1 kHz (CONTROL_PERIOD_TICKS).
 * ============================================================================= */
void control_task(void *arg)
{
    task_params_t *params = (task_params_t *)arg;

    ESP_ERROR_CHECK(mcpwm_setup());
    ESP_LOGI(TAG_PWM, "MCPWM setup complete");

    TickType_t last_wake_time = xTaskGetTickCount();

    // Configuracao do ADC1 (largura de conversao e atenuacao de entrada, iguais para todos os canais usados).
    adc1_config_width(ADC_WIDTH_CFG);
    for (int i = 0; i < SIG_COUNT; i++)
    {
        adc1_config_channel_atten(adc_sig_channel[i], ADC_ATTEN_CFG);
    }

    const float Ts = 1.0f / CONTROL_RATE_HZ; // Periodo de amostragem do LACO DE CONTROLE = 1 ms (nao confundir com
                                              // o periodo de PWM, que e 50 us). Usado no integrador de tensao,
                                              // no observador (kf_predict_states) e no fator de correcao de vies do Adam.

    const float C = C_nom;
    float L[PHASES_COUNT] = {0.0};
    float theta_sys[PHASES_COUNT][2] = {0};   // [fase][IDX_RL/IDX_VD]: parametros de planta identificados online.
    float m_sys[PHASES_COUNT][2] = {0};       // 1o momento do Adam (system params)
    float v_sys[PHASES_COUNT][2] = {0};       // 2o momento do Adam (system params)
    float theta_load[2] = {0};                // [IDX_G/IDX_IDC]: parametros do modelo de carga identificados online.
    const float theta_load_min[PARAMS_LOAD_COUNT] = {0.0f, 0.0f};
    const float theta_load_max[PARAMS_LOAD_COUNT] = {G_LOAD_MAX, IDC_LOAD_MAX};
    float m_load[2] = {0};
    float v_load[2] = {0};

    // Taxas de aprendizado (alpha) e coeficientes de momento (beta1/beta2) do otimizador Adam, separados para os
    // parametros de sistema (por fase) e de carga. alpha e escalado por Ts: e um "ganho por unidade de tempo",
    // nao por amostra, entao seu efeito pratico depende de Ts (isto e uma forma de discretizacao do gradiente).
    const float alpha_sys[PARAMS_SYS_COUNT] = {0.08f * Ts, 0.1f * Ts};
    const float beta1_sys = 0.94f;
    const float beta2_sys = 0.999f;
    const float alpha_load[PARAMS_LOAD_COUNT] = {1.0f * Ts, 100.0f * Ts};
    const float beta1_load = 0.94f;
    const float beta2_load = 0.99f;
    int adam_step = 0; // contador de passos, usado na correcao de vies do Adam (bias correction)

    const float theta_sys_min[PARAMS_SYS_COUNT] = {RL_MIN, VD_MIN};
    const float theta_sys_max[PARAMS_SYS_COUNT] = {RL_MAX, VD_MAX};
    for (int i = 0; i < PHASES_COUNT; i++)
    {
        L[i] = L_nom;
        theta_sys[i][IDX_RL] = RL_nom;
        theta_sys[i][IDX_VD] = VD_nom;
    }

    // Estados do observador de Kalman escalar (um par valor/derivada por grandeza medida/estimada).
    float z[SIG_COUNT] = {0.0};        // Medicoes (Z_COUNT medidas de fato + I_out estimado, se ADC_MEAS_IL_OUT==0)
    float x_pred[SIG_COUNT] = {0.0};   // Predicao a priori
    float x_dot_pred[SIG_COUNT] = {0.0};
    float x_hat[SIG_COUNT] = {0.0};    // Estimativa filtrada (a posteriori): usada por todo o resto do laco.
    float x_dot_hat[SIG_COUNT] = {0.0};
    float x_dot_model[PHASES_COUNT + 1] = {0.0}; // derivadas previstas pelo MODELO (nao pelo observador): uma por
                                                  // corrente de fase + uma para Vout.
    float duty[PHASES_COUNT] = {0.0f};     // duty efetivamente calculado e aplicado neste ciclo (fracao [0,1])
    float duty_bot[PHASES_COUNT] = {0.0f}; // copia de duty[] quando em modo boost (braço de baixo), para telemetria
                                            // e para o termo (1 - duty_bot) usado no residuo do identificador.
    float duty_top[PHASES_COUNT] = {0.0f}; // idem para o modo buck (braço de cima)
    float int_eV = 0.0f;                   // acumulador do integrador de tensao (estado do PI externo)

    // Covariancias do filtro de Kalman escalar: p11 = var(x), p22 = var(x_dot), p12 = cov(x, x_dot).
    float p11[SIG_COUNT] = {1.0};
    float p12[SIG_COUNT] = {0.0};
    float p22[SIG_COUNT] = {1.0};
    const float R[SIG_COUNT] = R_MEAS;
    const float Q_val[SIG_COUNT] = Q_VAL;
    const float Q_dot[SIG_COUNT] = Q_DOT;

    // [CRITICAL: item #2 do relatorio] Ganhos do PI/cascata FIXADOS EM 1.0: sao placeholders, NAO vem do
    // projeto por alocacao de polos ja pronto em Matlab/Cascade_gains_Boost.m (que recebe C, L, Vin, Vout_ref e
    // parametros de desempenho desejado k, ts, zeta e devolve os 3 ganhos corretos para a planta real). Mesmo
    // corrigindo o bug de unidade do duty (ver update_duty()), ganhos unitarios genericos muito provavelmente NAO
    // dao uma resposta estavel/bem amortecida para esta planta (C=12mF, L=80mH nominais): o mais provavel e
    // oscilacao, resposta muito lenta, ou saturacao persistente do duty.
    // CORRECAO SUGERIDA: rodar Cascade_gains_Boost.m com os parametros reais da bancada e substituir os valores
    // abaixo (ou expor os 3 ganhos por comando SPI para poder ajustar sem recompilar).
    float Kp_i = 1;
    float Kp_v = 1;
    float Ki_v = 1;

    while (1)
    {
        // ---------- 1) AQUISICAO ----------
        adc_read_states(z, Z_COUNT);

        // Se I_out nao e medido por ADC (ADC_MEAS_IL_OUT==0), estima-lo pelo modelo de carga do conversor
        // (balanco de corrente nos indutores menos a corrente no capacitor, ver model_estimate_iload em model.c).
        if (Z_COUNT < SIG_COUNT) // I_load is not measured, so we estimate it
        {
            z[SIG_I_OUT] = model_estimate_iload(x_hat, duty, x_dot_hat[SIG_V_OUT], C, params->pwm_mode);
        }
        // ---------- 2) OBSERVADOR (Kalman escalar) ----------
        // Funde a predicao do ciclo anterior (x_pred/x_dot_pred) com a nova medicao z[] -> x_hat/x_dot_hat.
        kf_update_states(x_pred, x_dot_pred, p11, p12, p22, z, R, x_hat, x_dot_hat, SIG_COUNT);

        // ---------- 3) MODELO ----------
        // Deriva x_dot_model a partir do modelo medio do conversor (boost ou buck) e dos parametros estimados
        // ATUAIS (theta_sys): usado como "verdade" para calcular o residuo do identificador logo abaixo.
        model_derivatives(x_dot_model, x_hat, duty, L, theta_sys, C, params->pwm_mode);

        // ---------- 4) IDENTIFICACAO ONLINE (MRAC + Adam) ----------
        adam_step++;
        // Fatores de correcao de vies do Adam (bias correction), dependentes do "tempo" decorrido (adam_step*Ts),
        // nao apenas do numero de passos: outra forma de tornar o otimizador ciente do periodo de amostragem.
        float bias_corr1_sys = 1.0f - powf(beta1_sys, adam_step * Ts);
        float bias_corr2_sys = 1.0f - powf(beta2_sys, adam_step * Ts);
        float bias_corr1_load = 1.0f - powf(beta1_load, adam_step * Ts);
        float bias_corr2_load = 1.0f - powf(beta2_load, adam_step * Ts);

        for (int i = 0; i < PHASES_COUNT; i++)
        {
            // Residuo entre a medicao filtrada da corrente e o que o modelo (com os theta_sys atuais) previa:
            // y_IL = Vin - Vout*(1-D) - L*dIL/dt_modelo  <->  y_IL deveria ser igual a R_L*IL + VD*(1-D) se o
            // modelo estivesse perfeito. phi_IL sao os "regressores" (IL e (1-duty_bot)) que multiplicam R_L e VD.
            float phi_IL[PARAMS_SYS_COUNT] = {x_hat[i], (1.0f - duty_bot[i])};
            float y_IL = x_hat[SIG_V_IN] - x_hat[SIG_V_OUT] * (1.0f - duty_bot[i]) - L[i] * x_dot_model[i];

            // Gradiente do erro quadratico em relacao a theta_sys[i] (minimos quadrados escalar).
            float grad[PARAMS_SYS_COUNT];
            compute_gradient_scalar(grad, phi_IL, y_IL, theta_sys[i], PARAMS_SYS_COUNT);

            // Passo do Adam: atualiza theta_sys[i] = [R_L, VD] na direcao que reduz o residuo.
            adam_update(theta_sys[i], m_sys[i], v_sys[i], grad,
                        alpha_sys, beta1_sys, beta2_sys, 1e-8f,
                        bias_corr1_sys, bias_corr2_sys, PARAMS_SYS_COUNT);

            // Impede que R_L/VD identificados saiam da faixa fisicamente plausivel (RL_MIN..MAX, VD_MIN..MAX).
            clamp_array(theta_sys[i], theta_sys_min, theta_sys_max, PARAMS_SYS_COUNT);
        }

        // Identificacao do modelo de carga: I_out = G*Vout + Idc, usando tanto o valor quanto a derivada de
        // Vout/I_out filtrados pelo observador como duas "medidas" simultaneas (2 equacoes, 2 incognitas).
        float phi_load[2][PARAMS_LOAD_COUNT] = {
            {x_hat[SIG_V_OUT], 1.0f},
            {x_dot_hat[SIG_V_OUT], 0.0f}};

        float y_load[2] = {
            x_hat[SIG_I_OUT],
            x_dot_hat[SIG_I_OUT]};

        float w_load[PARAMS_LOAD_COUNT] = {1.0f, 0.0003f}; // pesos relativos das duas "medidas" no gradiente
        float grad[PARAMS_LOAD_COUNT];
        compute_gradient(grad, (const float *)phi_load, y_load, theta_load, w_load, 2, PARAMS_LOAD_COUNT);

        adam_update(theta_load, m_load, v_load, grad,
                    alpha_load, beta1_load, beta2_load, 1e-8f,
                    bias_corr1_load, bias_corr2_load, PARAMS_LOAD_COUNT);

        clamp_array(theta_load, theta_load_min, theta_load_max, PARAMS_LOAD_COUNT);

        // ---------- 5) REFERENCIA DE CORRENTE (so no modo STANDALONE) ----------
#if OPERATION_MODE == OP_MODE_STANDALONE
        // Corrente de equilibrio que sustentaria Vout_ref em regime permanente, dado o modelo de carga estimado
        // (feedforward nao-linear: NAO existe equivalente no C-PI classico do artigo CBA2026).
        // [REVISAR: item #5 do relatorio] esta equacao quadratica fica mal-condicionada quando R_L (theta_sys)
        // e pequeno (a=sum(RL/(VD+Vout)) tende a 0), o que pode gerar IL_eq absurdamente grande logo no inicio,
        // antes de o MRAC convergir: nao ha soft-start em Vout_ref/IL_ref para amenizar isso.
        float IL_eq = compute_IL_eq(theta_sys, theta_load, x_hat, params->Vout_ref);

        // PI de tensao classico: proporcional + integral (Euler-avante) sobre o erro de Vout.
        // Estruturalmente equivalente ao laco externo do artigo CBA2026 (Kpv, Kiv), mas aqui SOMADO ao
        // feedforward nao-linear IL_eq acima: o artigo nao tem esse termo.
        float err_V = params->Vout_ref - x_hat[SIG_V_OUT];
        int_eV += Ts * err_V; // discretizacao por Euler-avante do integrador continuo (1/s -> Ts/(z-1))
                               // [REVISAR: item #4 do relatorio] sem limite (anti-windup): int_eV cresce
                               // livremente mesmo quando o duty ja esta saturado fisicamente.

        float IL_ref = IL_eq + Kp_v * err_V + Ki_v * int_eV;
#else
        float IL_ref = params->IL_ref; // modo INTEGRATED: referencia de corrente vem pronta de fora, via SPI.
#endif

        // ---------- 6) LEI DE CONTROLE ----------
        switch ((control_mode_t)params->ctrl_mode)
        {
        case CTRL_OPEN:
        {
            // Copia o duty manual recebido por SPI (protegido por secao critica, pois d_cmd e escrito pela
            // outra task/interrupcao) e o aplica diretamente, sem realimentacao.
            float d_cmd[PHASES_COUNT];
            taskENTER_CRITICAL(&params->telem_mux);
            memcpy(d_cmd, (const void *)params->d_cmd, sizeof(d_cmd));
            taskEXIT_CRITICAL(&params->telem_mux);
            duty_open(duty, d_cmd);
            break;
        }
        case CTRL_CASCADE:
            // Ver comentario detalhado na definicao de duty_cascade() acima (linearizacao por realimentacao,
            // NAO um PI classico; resultado em fracao [0,1]: cuidado com update_duty() logo mais abaixo).
            duty_cascade(duty, x_hat, L, theta_sys, IL_ref, Kp_i);
            break;
        default:
            // [CRITICAL: item #6 do relatorio] CTRL_IDA_PBC cai aqui (nao implementado) e o comentario original
            // diz "Unknown mode -> Keep at 0", mas NADA e feito: duty[] so e zerado uma vez, antes do while(1),
            // entao ao selecionar IDA-PBC o array mantem o ULTIMO valor calculado no ciclo anterior: o
            // conversor continua operando com o duty antigo, nao desliga. Se isso acontecer durante um
            // transitorio, e potencialmente perigoso.
            // CORRECAO SUGERIDA: "memset(duty, 0, sizeof(duty));" aqui, ou implementar IDA-PBC de fato.
            break;
        }

        // ---------- 7) ATUACAO ----------
        // [CRITICAL] nenhuma saturacao (clamp) de duty[] antes de aplicar: ver item #4 do relatorio. Recomenda-se
        // limitar duty[i] a uma faixa segura (ex.: 0.02 a 0.95, com margem para o tempo morto) logo aqui.
        update_duty(duty, params->pwm_mode); // ver aviso [CRITICAL] de unidade dentro de update_duty() acima.

        // Guarda o duty aplicado neste ciclo, separado por topologia (boost usa o braço de baixo / duty_bot;
        // buck usa o braço de cima / duty_top): usado no residuo do identificador (proximo ciclo) e na telemetria.
        if (params->pwm_mode == PWM_MODE_BOOST)
        {
            memcpy(duty_bot, duty, sizeof(duty));
            memset(duty_top, 0, sizeof(duty_top));
        }
        else
        {
            memset(duty_bot, 0, sizeof(duty_bot));
            memcpy(duty_top, duty, sizeof(duty));
        }

        // ---------- PREDICAO (observador) ----------
        // Propaga x_hat/x_dot_hat para o proximo ciclo (predicao a priori): nao depende de z, entao e feita
        // mesmo para I_out quando ele e estimado (nao medido).
        kf_predict_states(x_pred, x_dot_pred, x_hat, x_dot_hat, p11, p12, p22, Q_val, Q_dot, Ts, SIG_COUNT);

        // ---------- 8) TELEMETRIA ----------
        // Publica o estado atual para a task SPI escrava (communication.c), protegido por secao critica.
        taskENTER_CRITICAL(&params->telem_mux);
        memcpy((void *)params->x, x_hat, sizeof(x_hat));
        memcpy((void *)params->x_pred, x_pred, sizeof(x_pred));
        memcpy((void *)params->x_dot, x_dot_hat, sizeof(x_dot_hat));
        memcpy((void *)params->x_dot_model, x_dot_model, sizeof(x_dot_model));
        memcpy((void *)params->theta_sys, theta_sys, sizeof(theta_sys));
        memcpy((void *)params->theta_load, theta_load, sizeof(theta_load));
        memcpy((void *)params->d_bot, duty_bot, sizeof(duty_bot));
        memcpy((void *)params->d_top, duty_top, sizeof(duty_top));
        taskEXIT_CRITICAL(&params->telem_mux);

        // ---------- Ritmo do laco ----------
        // [CRITICAL: ver nota de topo do arquivo] CONTROL_PERIOD_TICKS = 1 ms. Isso significa que TODO o laco
        // acima (incluindo duty_cascade) so e reavaliado a cada 20 periodos de PWM (20 kHz / 1 kHz = 20). Para um
        // laco de CORRENTE isso e lento; vale medir na bancada se essa taxa e suficiente antes de gastar tempo
        // ajustando Kp_i.
        vTaskDelayUntil(&last_wake_time, CONTROL_PERIOD_TICKS);
    }
}
