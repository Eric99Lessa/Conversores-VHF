#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "config.h"

// Protocol version
#define FRAME_MAGIC 0xCAFE
#define FRAME_VERSION 0x01

typedef enum
{
    FRAME_TYPE_READ = 0x01,
    FRAME_TYPE_CMD = 0x02,
} frame_type_t;

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
    uint16_t magic;  // 0xCAFE — identifies a valid frame start
    uint8_t type;    // frame_type_t — identifies the frame content
    uint8_t version; // protocol version — future proofing
    uint32_t seq;    // sequence counter — detect dropped frames
} frame_header_t;

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

typedef struct
{
    frame_header_t header;
    uint8_t cmd_id; // which variable to update
    uint8_t index;  // for arrays: which element (0 if scalar)
    uint8_t index2; // for 2D arrays: second index (0 if not applicable)
    uint8_t reserved;
    float value; // new value
    uint16_t crc16;
} frame_cmd_t;
#pragma pack(pop)

void spi_slave_task(void *arg);
void spi_set_pending_cmd(const frame_cmd_t *cmd);
void spi_master_task(void *arg);

#endif // COMMUNICATION_H