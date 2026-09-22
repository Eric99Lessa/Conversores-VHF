#ifndef UART_TASK_H
#define UART_TASK_H

/**
 * FreeRTOS task: reads commands from UART0 and forwards them via
 * spi_set_pending_cmd().
 *
 * Supported commands (case-insensitive):
 *   M open|cascade|idapbc   — set control mode
 *   P boost|buck             — set PWM mode
 *   V <float>                — set Vout reference
 *   I <float>                — set IL reference
 *   D <float>                — set all phases duty (0-1000 permille)
 *   D <f0> <f1> <f2>         — set per-phase duty (queued individually)
 */
void uart_task(void *arg);

#endif // UART_TASK_H