/* =============================================================================
 * main.c  (ANOTADO)
 * =============================================================================
 * Ponto de entrada do firmware. Nao contem logica: so decide, em TEMPO DE
 * COMPILACAO (via #if COMMS_MODE == ..., config.h), qual par de tasks
 * FreeRTOS instanciar: ou seja, se ESTE binario, quando gravado num ESP32,
 * vai se comportar como o ESP DE CONTROLE ou o ESP DE COMUNICACAO. Os dois
 * papeis compartilham o mesmo codigo-fonte; o que muda e a macro COMMS_MODE
 * em config.h antes de compilar/gravar cada um dos dois ESP32 da bancada.
 * ============================================================================= */

#include "config.h"
#include "communication.h"

#if COMMS_MODE == COMMS_MODE_ON
#include "uart_task.h"
#else
#include "control.h"
#endif

// ===== APP MAIN =====
void app_main(void)
{
#if COMMS_MODE == COMMS_MODE_ON
    // ── ESP de Comunicacao ──────────────────────────────────────────────────
    // spi_master_task: fala com o ESP de controle via SPI (mestre) e imprime telemetria em CSV.
    // uart_task: le comandos do console serial e os enfileira para o spi_master_task enviar.
    xTaskCreatePinnedToCore(spi_master_task, "spi_task", 4096, NULL, PRIO_HIGH, NULL, CORE_MAIN);
    xTaskCreatePinnedToCore(uart_task, "uart_task", 4096, NULL, PRIO_LOW, NULL, CORE_SCND);
#else
    // ── ESP de Controle ──────────────────────────────────────────────────
    // task_params_t compartilhada entre control_task (produtor do estado/telemetria) e spi_slave_task
    // (consumidor da telemetria, produtor de comandos vindos de fora). Estado inicial seguro: modo de controle
    // CTRL_OPEN (malha aberta) e topologia PWM_MODE_BOOST.
    static task_params_t params = {
        .telem_mux = portMUX_INITIALIZER_UNLOCKED,
        .pwm_mode = PWM_MODE_BOOST,
        .ctrl_mode = CTRL_OPEN,
    };

    // control_task: laco de controle a 1 kHz, prioridade ALTA, fixado no core 0 (tempo real).
    // spi_slave_task: responde as transacoes SPI do ESP de comunicacao, prioridade BAIXA, no core 1.
    xTaskCreatePinnedToCore(control_task, "ctrl", 4096, &params, PRIO_HIGH, NULL, CORE_MAIN);
    xTaskCreatePinnedToCore(spi_slave_task, "spi_task", 4096, &params, PRIO_LOW, NULL, CORE_SCND);
#endif
}
