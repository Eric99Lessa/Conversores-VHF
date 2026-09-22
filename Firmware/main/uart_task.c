#include "uart_task.h"
#include "communication.h" // frame_cmd_t, cmd_id_t, frame_type_t
#include "config.h"        // control_mode_t, pwm_mode_t

#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>

#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "UART_TASK";

// ─── Frame builder ────────────────────────────────────────────────────────────

static void make_cmd(frame_cmd_t *f, cmd_id_t id, uint8_t idx, uint8_t idx2, float value)
{
    memset(f, 0, sizeof(*f));
    f->header.magic = FRAME_MAGIC;
    f->header.type = FRAME_TYPE_CMD;
    f->header.version = FRAME_VERSION;
    f->cmd_id = (uint8_t)id;
    f->index = idx;
    f->index2 = idx2;
    f->value = value;
    // CRC is finalised inside spi_set_pending_cmd()
}

// ─── Command parsers ──────────────────────────────────────────────────────────

/** "M open|cascade|idapbc" → CMD_CTRL_MODE */
static bool parse_m_command(const char *line, frame_cmd_t *out)
{
    while (*line && isspace((unsigned char)*line))
        line++;
    if (*line != 'M' && *line != 'm')
        return false;
    line++;
    while (*line && isspace((unsigned char)*line))
        line++;
    if (!*line)
        return false;

    control_mode_t mode;
    if (strncasecmp(line, "open", 4) == 0)
        mode = CTRL_OPEN;
    else if (strncasecmp(line, "cascade", 7) == 0)
        mode = CTRL_CASCADE;
    else if (strncasecmp(line, "idapbc", 6) == 0)
        mode = CTRL_IDA_PBC;
    else
    {
        printf("CMD_ERR: Unknown mode. Use open|cascade|idapbc\n");
        return false;
    }

    make_cmd(out, CMD_CTRL_MODE, 0, 0, (float)mode);
    return true;
}

/** "P boost|buck" → CMD_PWM_MODE */
static bool parse_p_command(const char *line, frame_cmd_t *out)
{
    while (*line && isspace((unsigned char)*line))
        line++;
    if (*line != 'P' && *line != 'p')
        return false;
    line++;
    while (*line && isspace((unsigned char)*line))
        line++;
    if (!*line)
        return false;

    pwm_mode_t mode;
    if (strncasecmp(line, "boost", 5) == 0)
        mode = PWM_MODE_BOOST;
    else if (strncasecmp(line, "buck", 4) == 0)
        mode = PWM_MODE_BUCK;
    else
    {
        printf("CMD_ERR: Unknown PWM mode. Use boost|buck\n");
        return false;
    }

    make_cmd(out, CMD_PWM_MODE, 0, 0, (float)mode);
    return true;
}

/** "V <float>" → CMD_VOUT_REF */
static bool parse_v_command(const char *line, frame_cmd_t *out)
{
    while (*line && isspace((unsigned char)*line))
        line++;
    if (*line != 'V' && *line != 'v')
        return false;
    line++;
    while (*line && isspace((unsigned char)*line))
        line++;
    if (!*line)
        return false;

    char *end = NULL;
    float val = strtof(line, &end);
    if (end == line)
    {
        printf("CMD_ERR: Invalid float for V command\n");
        return false;
    }

    make_cmd(out, CMD_VOUT_REF, 0, 0, val);
    return true;
}

/** "I <float>" → CMD_IL_REF */
static bool parse_i_command(const char *line, frame_cmd_t *out)
{
    while (*line && isspace((unsigned char)*line))
        line++;
    if (*line != 'I' && *line != 'i')
        return false;
    line++;
    while (*line && isspace((unsigned char)*line))
        line++;
    if (!*line)
        return false;

    char *end = NULL;
    float val = strtof(line, &end);
    if (end == line)
    {
        printf("CMD_ERR: Invalid float for I command\n");
        return false;
    }

    make_cmd(out, CMD_IL_REF, 0, 0, val);
    return true;
}

/**
 * "D <float>"           → CMD_DUTY, index=0xFF (all phases)
 * "D <f0> <f1> <f2>"   → CMD_DUTY, index=0/1/2 (per-phase, queued individually)
 * Returns number of frames written into out[] (1 or 3), or 0 on error.
 */
static int parse_d_command(const char *line, frame_cmd_t out[3])
{
    while (*line && isspace((unsigned char)*line))
        line++;
    if (*line != 'D' && *line != 'd')
        return 0;
    line++;
    while (*line && isspace((unsigned char)*line))
        line++;
    if (!*line)
        return 0;

    float vals[3];
    int n = 0;
    const char *p = line;
    while (n < 3 && *p)
    {
        char *end;
        float v = strtof(p, &end);
        if (end == p)
            break;
        vals[n++] = v;
        p = end;
        while (*p && isspace((unsigned char)*p))
            p++;
    }

    if (n == 1)
    {
        make_cmd(&out[0], CMD_DUTY, 0xFF, 0, vals[0]);
        return 1;
    }
    else if (n == 3)
    {
        for (int i = 0; i < 3; i++)
            make_cmd(&out[i], CMD_DUTY, (uint8_t)i, 0, vals[i]);
        return 3;
    }

    printf("CMD_ERR: D requires 1 or 3 float values\n");
    return 0;
}

// ─── Task ─────────────────────────────────────────────────────────────────────

void uart_task(void *arg)
{
    (void)arg;

    const uart_port_t U = UART_NUM_0;

    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(U, &cfg));
    ESP_ERROR_CHECK(uart_driver_install(U, 1024, 0, 0, NULL, 0));

    ESP_LOGI(TAG, "Console ready. Commands:");
    ESP_LOGI(TAG, "  M open|cascade|idapbc  — control mode");
    ESP_LOGI(TAG, "  P boost|buck           — PWM mode");
    ESP_LOGI(TAG, "  V <float>              — Vout reference");
    ESP_LOGI(TAG, "  I <float>              — IL reference");
    ESP_LOGI(TAG, "  D <float>              — all phases duty (permille)");
    ESP_LOGI(TAG, "  D <f0> <f1> <f2>       — per-phase duty");

    char line[128];
    int idx = 0;

    while (1)
    {
        uint8_t ch;
        if (uart_read_bytes(U, &ch, 1, pdMS_TO_TICKS(100)) <= 0)
            continue;

        if (ch == '\r' || ch == '\n')
        {
            if (idx == 0)
                continue;
            line[idx] = '\0';
            idx = 0;

            frame_cmd_t cmd;
            bool ok = false;

            if (parse_m_command(line, &cmd))
                ok = true;
            else if (parse_p_command(line, &cmd))
                ok = true;
            else if (parse_v_command(line, &cmd))
                ok = true;
            else if (parse_i_command(line, &cmd))
                ok = true;
            else
                ESP_LOGW(TAG, "CMD_ERR: Use M|P|V|I — type ? for help");

            if (ok)
            {
                spi_set_pending_cmd(&cmd);
                ESP_LOGI(TAG, "CMD_OK: id=0x%02X val=%.4f", cmd.cmd_id, cmd.value);
            }
            continue;
        }

        if (ch == 0x08 || ch == 0x7F)
        {
            if (idx > 0)
                idx--;
            continue;
        }

        if (idx < (int)sizeof(line) - 1)
            line[idx++] = (char)ch;
        else
        {
            ESP_LOGW(TAG, "Line buffer overflow, resetting");
            idx = 0;
        }
    }
}