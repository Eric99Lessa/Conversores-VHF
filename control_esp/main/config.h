#ifndef CONFIG_H
#define CONFIG_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/mcpwm.h"

typedef enum
{
    COMMS_MODE_OFF = 0, // CONTROL ESP
    COMMS_MODE_ON       // COMMS ESP
} comms_mode_t;

typedef enum
{
    OP_MODE_STANDALONE = 0, // converter runs alone
    OP_MODE_INTEGRATED      // converter is part of a larger system
} operation_mode_t;

#define COMMS_MODE COMMS_MODE_OFF
#define OPERATION_MODE OP_MODE_STANDALONE

// PWM
#define PWM_FREQUENCY_HZ 20000
#define PWM_INIT_DUTY_PC 0.0f
#define PWM_UNIT MCPWM_UNIT_0
#define PHASES_COUNT 3

// Parameters
#define C_nom 0.012f
#define L_nom 0.08f
#define RL_nom 0.001f
#define VD_nom 0.8f

// Minimum and maximum values
#define RL_MIN 0.001f
#define RL_MAX 1.0f
#define VD_MIN 0.7f
#define VD_MAX 2.0f

// Minimum and maximum values for load id
#define G_LOAD_MAX 2.0f
#define IDC_LOAD_MAX 150.0f

// GPÌO
#define U_L_GPIO (22)
#define V_L_GPIO (21)
#define W_L_GPIO (5)
#define U_U_GPIO (23)
#define V_U_GPIO (3)
#define W_U_GPIO (18)
#define U_ERR_GPIO (1)
#define V_ERR_GPIO (19)
#define W_ERR_GPIO (17)

// CONTROL PERIOD
#define CONTROL_RATE_HZ 1000
#define CONTROL_PERIOD_TICKS pdMS_TO_TICKS(1000 / CONTROL_RATE_HZ)

// ADC
#define ADC_SAMPLES_PER_CH 4
#define ADC_WIDTH_CFG ADC_WIDTH_BIT_12
#define ADC_ATTEN_CFG ADC_ATTEN_DB_11
#define ADC_MEAS_IL_OUT 0 // 0 to false and 1 to true
#define ADC_THR 2890      // threshold in ADC reading to measurement conversion

// SPI pins/host
#define PIN_MISO 12
#define PIN_MOSI 13
#define PIN_SCK 14
#define PIN_CS 27

// TASK CORES / PRIORITIES
#define CORE_MAIN 0
#define CORE_SCND 1
#define PRIO_LOW 0
#define PRIO_HIGH 1

// PWM mode selection
typedef enum
{
    PWM_MODE_BOOST = 0,
    PWM_MODE_BUCK // 1
} pwm_mode_t;

typedef enum
{
    CTRL_OPEN = 0,
    CTRL_CASCADE,
    CTRL_IDA_PBC
} control_mode_t;

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

typedef enum
{
    IDX_RL = 0,
    IDX_VD,
    PARAMS_SYS_COUNT
} params_sys_idx_t;

typedef enum
{
    IDX_G = 0,
    IDX_IDC,
    PARAMS_LOAD_COUNT
} params_load_idx_t;

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

#define Z_COUNT (SIG_COUNT - 1 + ADC_MEAS_IL_OUT)

extern const adc_channel_t adc_sig_channel[SIG_COUNT];

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

#define R_MEAS {         \
    [SIG_IL1] = 0.01f,   \
    [SIG_IL2] = 0.01f,   \
    [SIG_IL3] = 0.01f,   \
    [SIG_V_OUT] = 0.01f, \
    [SIG_V_IN] = 0.01f,  \
    [SIG_I_OUT] = 0.01f, \
}

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

typedef struct
{
    portMUX_TYPE telem_mux;
    volatile uint8_t pwm_mode;  // e.g. PWM_MODE_BOOST
    volatile uint8_t ctrl_mode; // updated by SPI task at runtime
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