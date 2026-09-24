/* =============================================================================
 * uart_task.h  (ANOTADO)
 * =============================================================================
 * FreeRTOS task: le comandos de texto da UART0 e os encaminha via
 * spi_set_pending_cmd() (communication.c) para serem transmitidos ao ESP
 * de controle na proxima transacao SPI.
 *
 * Comandos suportados pelo PARSER (case-insensitive):
 *   M open|cascade|idapbc  : define o modo de controle
 *   P boost|buck            : define o modo PWM
 *   V <float>               : define a referencia de Vout
 *   I <float>               : define a referencia de IL
 *   D <float>               : define o duty manual de todas as fases (0-1000 permille, ver ressalva abaixo)
 *   D <f0> <f1> <f2>        : define o duty manual por fase (enfileirado individualmente)
 *
 * [CRITICAL] Apesar de implementado no parser (parse_d_command, uart_task.c) e documentado aqui e no texto de
 * ajuda do console, o comando "D" NUNCA E DESPACHADO no laco principal de uart_task(): ver uart_task.c para o
 * detalhe e a correcao sugerida. Do jeito que esta, digitar "D <valor>" no console retorna erro
 * "CMD_ERR: Use M|P|V|I", mesmo a funcao de parse existindo e sendo correta.
 * ============================================================================= */

#ifndef UART_TASK_H
#define UART_TASK_H

void uart_task(void *arg);

#endif // UART_TASK_H
