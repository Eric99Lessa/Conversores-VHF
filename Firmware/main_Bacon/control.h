/* =============================================================================
 * control.h  (ANOTADO): expoe apenas a task de controle. Ver control.c
 * para a explicacao completa do laco (1 kHz), das leis CTRL_OPEN/CASCADE/
 * IDA_PBC e das marcacoes [CRITICAL]/[REVISAR] encontradas na revisao.
 * ============================================================================= */

#ifndef CONTROL_H
#define CONTROL_H

void control_task(void *arg);

#endif
