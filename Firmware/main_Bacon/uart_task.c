/* =============================================================================
 * uart_task.c  (ANOTADO)
 * =============================================================================
 * Console de texto sobre UART0 (115200 8N1) para o ESP DE COMUNICACAO.
 * Le uma linha por vez (terminada em \r ou \n), tenta cada parser de
 * comando em sequencia (M/P/V/I/D), monta um frame_cmd_t (communication.h)
 * e o enfileira via spi_set_pending_cmd() para ser enviado ao ESP de
 * controle na proxima transacao SPI (communication.c).
 *
 * [CRITICAL: item #6 do relatorio] o comando "D" (duty manual) esta
 * TOTALMENTE IMPLEMENTADO em parse_d_command() logo abaixo, e ate aparece
 * no texto de ajuda impresso ao iniciar a task, MAS NUNCA E CHAMADO no
 * laco de despacho dentro de uart_task() (ver a cadeia de if/else if perto
 * do fim deste arquivo). Um operador que digitar "D 30" no console recebe
 * "CMD_ERR: Use M|P|V|I", como se o comando nao existisse. Isso so afeta o
 * CONSOLE UART: o mesmo comando (CMD_DUTY) chega normalmente ao ESP de
 * controle se for montado e enviado via SPI por outro caminho (ex.: um
 * script Python falando diretamente o protocolo binario). Se o Eric conta
 * com o console serial para testar duty manual antes de fechar a malha,
 * este bug pode ter mascarado testes de CTRL_OPEN.
 * ============================================================================= */

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

// ─── Montador de frame ────────────────────────────────────────────────────────

// Preenche um frame_cmd_t com cabecalho + campos do comando; o CRC e calculado depois, dentro de
// spi_set_pending_cmd() (communication.c), entao aqui ele fica zerado (memset inicial).
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
    // CRC e finalizado dentro de spi_set_pending_cmd()
}

// ─── Parsers de comando ──────────────────────────────────────────────────────

/** "M open|cascade|idapbc" -> CMD_CTRL_MODE
 * [CRITICAL] "idapbc" e aceito AQUI (o parser nao valida se o modo esta implementado), mas control_task() em
 * control.c nao tem "case CTRL_IDA_PBC" no switch: selecionar esse modo cai no "default" (mantem o ultimo duty
 * calculado, nao desliga o conversor). Ver control.c. */
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

/** "P boost|buck" -> CMD_PWM_MODE */
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

/** "V <float>" -> CMD_VOUT_REF */
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

/** "I <float>" -> CMD_IL_REF */
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
 * "D <float>"           -> CMD_DUTY, index=0xFF (todas as fases)
 * "D <f0> <f1> <f2>"    -> CMD_DUTY, index=0/1/2 (por fase, enfileirados individualmente)
 * Retorna o numero de frames escritos em out[] (1 ou 3), ou 0 em erro.
 *
 * [CRITICAL] Esta funcao esta CORRETA e completa, mas veja a nota de topo do arquivo e o laco de despacho em
 * uart_task() logo abaixo: ela NUNCA E CHAMADA. E codigo morto do ponto de vista do console UART.
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

/* uart_task(): configura a UART0 (115200 8N1) e entra num laco que le byte a byte, monta uma linha ate \r/\n,
 * tenta cada parser de comando em ordem e, se reconhecido, enfileira via spi_set_pending_cmd(). Trata backspace
 * (0x08/0x7F) apagando o ultimo caractere do buffer local, e protege contra overflow do buffer de linha (128
 * bytes) reiniciando-o com um aviso. */
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
    // [REVISAR: item #8 do relatorio] o texto diz "permille" (0-1000), mas nao ha nenhuma conversao de escala em
    // parse_d_command() nem em process_cmd_frame() (communication.c): o valor digitado e usado como veio. Se a
    // intencao real e bater com mcpwm_set_duty() (0-100, PORCENTAGEM, ver control.c), o texto deveria dizer
    // "percent", nao "permille". E academico enquanto o comando nem e despachado (ver [CRITICAL] abaixo).
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

            // [CRITICAL: item #6 do relatorio] falta "else if (parse_d_command(...))" nesta cadeia. O comando
            // "D", documentado acima e totalmente implementado em parse_d_command(), nunca e testado aqui, entao
            // qualquer linha comecando com "D"/"d" cai direto no "else" e imprime o erro generico abaixo.
            // CORRECAO SUGERIDA (esta funcao so aceita 1 frame por vez do jeito que esta escrita; parse_d_command
            // pode devolver 1 OU 3 frames, entao o encaminhamento precisa de um pequeno laco, por exemplo:
            //
            //   frame_cmd_t d_cmds[3];
            //   int n_d = parse_d_command(line, d_cmds);
            //   if (n_d > 0) {
            //       for (int k = 0; k < n_d; k++) spi_set_pending_cmd(&d_cmds[k]);
            //       ESP_LOGI(TAG, "CMD_OK: D (%d frame(s))", n_d);
            //       continue; // ja tratado, pula o "ok = true" generico abaixo
            //   }
            if (parse_m_command(line, &cmd))
                ok = true;
            else if (parse_p_command(line, &cmd))
                ok = true;
            else if (parse_v_command(line, &cmd))
                ok = true;
            else if (parse_i_command(line, &cmd))
                ok = true;
            else
                ESP_LOGW(TAG, "CMD_ERR: Use M|P|V|I — type ? for help"); // nota: a mensagem nem menciona "D"

            if (ok)
            {
                spi_set_pending_cmd(&cmd);
                ESP_LOGI(TAG, "CMD_OK: id=0x%02X val=%.4f", cmd.cmd_id, cmd.value);
            }
            continue;
        }

        if (ch == 0x08 || ch == 0x7F) // backspace / DEL
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
