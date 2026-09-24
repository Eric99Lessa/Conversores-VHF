/* =============================================================================
 * communication.h  (ANOTADO)
 * =============================================================================
 * Define o protocolo binario trocado por SPI entre os dois ESP32
 * (controle <-> comunicacao): cabecalho com magic/versao/sequencia, CRC16
 * ao final de cada frame, e dois tipos de frame:
 *   - frame_read_t:  ESP de controle -> ESP de comunicacao (telemetria)
 *   - frame_cmd_t:   ESP de comunicacao -> ESP de controle (comando)
 * Os campos usam #pragma pack(push,1) para garantir layout identico dos
 * dois lados do link SPI (sem padding do compilador).
 * ============================================================================= */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "config.h"

// Versao do protocolo: trocar aqui e recompilar OS DOIS lados (controle e comunicacao) se o layout mudar.
#define FRAME_MAGIC 0xCAFE
#define FRAME_VERSION 0x01

typedef enum
{
    FRAME_TYPE_READ = 0x01,
    FRAME_TYPE_CMD = 0x02,
} frame_type_t;

// Identificador de QUAL variavel um frame_cmd_t esta atualizando.
// [CRITICAL: ver uart_task.c] CMD_DUTY (0x05) e alcancavel via SPI (communication.c, process_cmd_frame), mas o
// comando de console "D" que deveria gerar este mesmo cmd_id nunca e despachado em uart_task.c.
typedef enum
{
    CMD_CTRL_MODE = 0x01,
    CMD_PWM_MODE = 0x02,
    CMD_VOUT_REF = 0x03,
    CMD_IL_REF = 0x04,
    CMD_DUTY = 0x05
} cmd_id_t;

#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;  // 0xCAFE: identifica o inicio de um frame valido
    uint8_t type;    // frame_type_t: identifica o conteudo do frame
    uint8_t version; // versao do protocolo: para evolucao futura
    uint32_t seq;    // contador de sequencia: permite detectar frames perdidos
} frame_header_t;

// Frame de TELEMETRIA (controle -> comunicacao): espelha o essencial de task_params_t (estado estimado, derivadas
// do modelo, parametros identificados, duty aplicado por topologia).
typedef struct
{
    frame_header_t header;
    float z[Z_COUNT];
    float x[SIG_COUNT];
    float x_pred[SIG_COUNT];
    float x_dot[SIG_COUNT];
    float x_dot_model[PHASES_COUNT + 1];
    float theta_sys[PHASES_COUNT][PARAMS_SYS_COUNT];
    float theta_load[PARAMS_LOAD_COUNT];
    float d_bot[PHASES_COUNT];
    float d_top[PHASES_COUNT];
    uint16_t crc16;
} frame_read_t;

// Frame de COMANDO (comunicacao -> controle): atualiza uma variavel por vez (cmd_id + index/index2 + value).
typedef struct
{
    frame_header_t header;
    uint8_t cmd_id; // qual variavel atualizar (cmd_id_t)
    uint8_t index;  // para arrays: qual elemento (0 se escalar; 0xFF em CMD_DUTY significa "todas as fases")
    uint8_t index2; // para arrays 2D: segundo indice (nao usado atualmente por nenhum cmd_id)
    uint8_t reserved;
    float value; // novo valor
    uint16_t crc16;
} frame_cmd_t;
#pragma pack(pop)

void spi_slave_task(void *arg);       // roda no ESP de controle: publica telemetria e aplica comandos recebidos
void spi_set_pending_cmd(const frame_cmd_t *cmd); // enfileira um comando para ser enviado no proximo ciclo SPI (mestre)
void spi_master_task(void *arg);      // roda no ESP de comunicacao: troca frames por SPI e imprime telemetria em CSV

#endif // COMMUNICATION_H
