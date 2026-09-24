/* =============================================================================
 * communication.c  (ANOTADO)
 * =============================================================================
 * Implementa os DOIS lados do link SPI entre os dois ESP32:
 *   - spi_slave_task():  roda no ESP DE CONTROLE. A cada transacao SPI
 *     (iniciada pelo mestre), envia um frame_read_t com a telemetria atual
 *     e recebe simultaneamente um frame_cmd_t (full-duplex), que e validado
 *     e aplicado a task_params_t via process_cmd_frame().
 *   - spi_master_task(): roda no ESP DE COMUNICACAO. Periodicamente (ou
 *     imediatamente, se houver comando na fila) troca um frame com o
 *     escravo, valida o frame_read_t recebido e imprime a telemetria em
 *     CSV (linha "TELEM,...") para captura em bancada.
 * A fila s_cmd_queue (preenchida por uart_task.c via spi_set_pending_cmd())
 * desacopla a chegada de comandos do console UART do ritmo fixo do SPI.
 * ============================================================================= */

#include "communication.h"
#include "esp_log.h"
#include "esp_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <inttypes.h>
#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "esp_heap_caps.h"

// Tamanho de transacao SPI = o maior dos dois tipos de frame (os dois lados sempre trocam blocos deste tamanho,
// mesmo que um dos frames seja menor: o restante e preenchido com zeros).
#define SPI_XFER_BYTES \
    (sizeof(frame_read_t) > sizeof(frame_cmd_t) ? sizeof(frame_read_t) : sizeof(frame_cmd_t))

#define SPI_POLL_PERIOD_MS 100 // periodo de pooling do mestre quando nao ha comando pendente na fila

#if COMMS_MODE == COMMS_MODE_ON
#define TAG_SPI "SPI_MASTER"
#else
#define TAG_SPI "SPI_SLAVE"
#endif

static uint16_t frame_crc16(const void *data, size_t len)
{
    return esp_crc16_le(0, (const uint8_t *)data, len);
}

// Confere o CRC16 de um frame_cmd_t recebido (calculado sobre todo o frame exceto o proprio campo crc16).
static bool cmd_crc_ok(const frame_cmd_t *f)
{
    uint16_t expected = frame_crc16(f, sizeof(frame_cmd_t) - sizeof(f->crc16));
    return f->crc16 == expected;
}

// Monta um frame_read_t com a telemetria atual, lendo task_params_t em secao critica (protegido contra a task
// de controle escrever no meio da copia) e fechando com o CRC16.
static void build_read_frame(frame_read_t *f, task_params_t *params, uint32_t seq)
{
    memset(f, 0, sizeof(*f));

    f->header.magic = FRAME_MAGIC;
    f->header.type = FRAME_TYPE_READ;
    f->header.version = FRAME_VERSION;
    f->header.seq = seq;

    taskENTER_CRITICAL(&params->telem_mux);
    memcpy(f->z, (const void *)params->z, sizeof(f->z));
    memcpy(f->x, (const void *)params->x, sizeof(f->x));
    memcpy(f->x_pred, (const void *)params->x_pred, sizeof(f->x_pred));
    memcpy(f->x_dot, (const void *)params->x_dot, sizeof(f->x_dot));
    memcpy(f->x_dot_model, (const void *)params->x_dot_model, sizeof(f->x_dot_model));
    memcpy(f->theta_sys, (const void *)params->theta_sys, sizeof(f->theta_sys));
    memcpy(f->theta_load, (const void *)params->theta_load, sizeof(f->theta_load));
    memcpy(f->d_bot, (const void *)params->d_bot, sizeof(f->d_bot));
    memcpy(f->d_top, (const void *)params->d_top, sizeof(f->d_top));
    taskEXIT_CRITICAL(&params->telem_mux);

    f->crc16 = frame_crc16(f, sizeof(*f) - sizeof(f->crc16));
}

/* process_cmd_frame(): valida (magic/tipo/versao/CRC) e aplica um frame_cmd_t recebido do mestre a task_params_t,
 * dentro de uma secao critica (a task de controle pode estar lendo esses mesmos campos a qualquer momento). Esta
 * e a UNICA porta de entrada de comandos no ESP de controle: tanto faz se o comando se originou do console UART
 * (uart_task.c -> spi_set_pending_cmd) ou de outro mestre SPI qualquer. */
static void process_cmd_frame(const frame_cmd_t *f, task_params_t *params)
{
    if (f->header.magic != FRAME_MAGIC)
        return;
    if (f->header.type != FRAME_TYPE_CMD)
        return;
    if (f->header.version != FRAME_VERSION)
        return;
    if (!cmd_crc_ok(f))
        return;

    taskENTER_CRITICAL(&params->telem_mux);
    switch ((cmd_id_t)f->cmd_id)
    {
    case CMD_CTRL_MODE:
        params->ctrl_mode = (uint8_t)f->value;
        break;
    case CMD_PWM_MODE:
        params->pwm_mode = (uint8_t)f->value;
        break;
    case CMD_VOUT_REF:
        params->Vout_ref = f->value;
        break;
    case CMD_IL_REF:
        params->IL_ref = f->value;
        break;
    case CMD_DUTY:
        // index==0xFF: aplica o mesmo valor as 3 fases; caso contrario, atualiza so a fase "index".
        // Observacao: escreve em d_bot (usado por duty_open() no modo boost); em modo buck isso NAO afeta
        // d_top, ou seja, o comando manual de duty so tem efeito pratico com pwm_mode==PWM_MODE_BOOST.
        if (f->index == 0xFF)
        {
            for (int i = 0; i < PHASES_COUNT; i++)
                params->d_bot[i] = f->value;
        }
        else if (f->index < PHASES_COUNT)
        {
            params->d_bot[f->index] = f->value;
        }
        break;
    default:
        break;
    }
    taskEXIT_CRITICAL(&params->telem_mux);

    ESP_LOGI(TAG_SPI, "CMD received: id=0x%02X idx=%u idx2=%u val=%.4f",
             f->cmd_id, f->index, f->index2, f->value);
}

/* -----------------------------------------------------------------------
 * spi_slave_init_simple(): configura o ESP como ESCRAVO SPI (VSPI_HOST),
 * liberando antes os pinos U0TXD/U0RXD do UART0 (que compartilham GPIO
 * 1/3 com MOSI/MISO nesta placa: por isso o UART e desabilitado no ESP
 * de controle, que so usa SPI).
 * ----------------------------------------------------------------------- */
static void spi_slave_init_simple(void)
{
    if (uart_is_driver_installed(UART_NUM_0))
        uart_driver_delete(UART_NUM_0);

    gpio_reset_pin(GPIO_NUM_1); // U0TXD
    gpio_reset_pin(GPIO_NUM_3); // U0RXD

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_XFER_BYTES,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 2,
        .flags = 0,
    };

    gpio_set_pull_mode(PIN_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_SCK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_CS, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG_SPI, "SPI slave ready. XFER=%u READ=%u CMD=%u",
             (unsigned)SPI_XFER_BYTES,
             (unsigned)sizeof(frame_read_t),
             (unsigned)sizeof(frame_cmd_t));
}

/* spi_slave_task(): laco principal do ESCRAVO SPI (ESP de controle). Cada iteracao monta a telemetria atual,
 * executa UMA transacao full-duplex bloqueante (spi_slave_transmit, espera indefinidamente o mestre iniciar o
 * clock) e processa o que veio no rx_buf como um possivel comando. Usa buffers alocados em memoria capaz de DMA
 * (heap_caps_malloc com MALLOC_CAP_DMA), exigencia do driver SPI slave do ESP-IDF. */
void spi_slave_task(void *arg)
{
    task_params_t *params = (task_params_t *)arg;

    spi_slave_init_simple();

    uint8_t *tx_dma = heap_caps_malloc(SPI_XFER_BYTES, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(SPI_XFER_BYTES, MALLOC_CAP_DMA);

    if (!tx_dma || !rx_buf)
    {
        ESP_LOGE(TAG_SPI, "DMA buffer alloc failed");
        vTaskDelete(NULL);
    }

    uint32_t seq = 0;

    while (1)
    {
        frame_read_t tx;
        build_read_frame(&tx, params, seq++);

        memset(tx_dma, 0, SPI_XFER_BYTES);
        memcpy(tx_dma, &tx, sizeof(tx));
        memset(rx_buf, 0, SPI_XFER_BYTES);

        spi_slave_transaction_t t = {
            .length = 8 * SPI_XFER_BYTES,
            .tx_buffer = tx_dma,
            .rx_buffer = rx_buf,
        };

        esp_err_t err = spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY); // bloqueia ate o mestre iniciar a transacao
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_SPI, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            continue;
        }

        process_cmd_frame((const frame_cmd_t *)rx_buf, params);
    }
}

// ===================== Lado MESTRE (ESP de comunicacao) =====================

static spi_device_handle_t s_spi;

#define CMD_QUEUE_LEN 8
static QueueHandle_t s_cmd_queue = NULL; // fila de comandos pendentes, preenchida por uart_task.c (console)

// ── CRC helpers ───────────────────────────────────────────────────────────────

static bool read_frame_crc_ok(const frame_read_t *f)
{
    uint16_t expected = frame_crc16(f, sizeof(*f) - sizeof(f->crc16));
    return f->crc16 == expected;
}

static void cmd_frame_finalize(frame_cmd_t *f)
{
    f->crc16 = frame_crc16(f, sizeof(*f) - sizeof(f->crc16));
}

// ── API publica ────────────────────────────────────────────────────────────────

static void spi_master_init(void)
{
    s_cmd_queue = xQueueCreate(CMD_QUEUE_LEN, sizeof(frame_cmd_t));
    configASSERT(s_cmd_queue);

    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_XFER_BYTES,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2 * 1000 * 1000, // 2 MHz
        .mode = 0,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
        .flags = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &s_spi));

    ESP_LOGI(TAG_SPI, "SPI master ready. XFER=%u READ=%u CMD=%u",
             (unsigned)SPI_XFER_BYTES,
             (unsigned)sizeof(frame_read_t),
             (unsigned)sizeof(frame_cmd_t));
}

/* spi_set_pending_cmd(): chamada por uart_task.c quando um comando de console valido e reconhecido. Finaliza o
 * CRC e enfileira o frame para ser enviado na PROXIMA transacao SPI (nao envia imediatamente: o mestre so fala
 * com o escravo dentro de spi_master_task). Se a fila estiver cheia, o comando e DESCARTADO silenciosamente
 * (so um log de warning), sem retry. */
void spi_set_pending_cmd(const frame_cmd_t *cmd)
{
    frame_cmd_t copy = *cmd;
    cmd_frame_finalize(&copy);

    if (xQueueSend(s_cmd_queue, &copy, pdMS_TO_TICKS(50)) != pdTRUE)
        ESP_LOGW(TAG_SPI, "CMD queue full, command dropped");
}

// ── Troca SPI ──────────────────────────────────────────────────────────────────

/* spi_exchange(): executa UMA transacao SPI full-duplex. Se houver um comando na fila, ele e enviado nesta
 * transacao (tx_buf); caso contrario, envia um frame vazio (so zeros, que process_cmd_frame no escravo vai
 * rejeitar por magic invalido). O que volta em rx_buf e sempre copiado para telem_out (validacao fica a cargo
 * do chamador). */
static esp_err_t spi_exchange(frame_read_t *telem_out)
{
    static uint8_t tx_buf[SPI_XFER_BYTES];
    static uint8_t rx_buf[SPI_XFER_BYTES];

    memset(tx_buf, 0, sizeof(tx_buf));
    memset(rx_buf, 0, sizeof(rx_buf));

    // Envia o proximo comando enfileirado, se houver; senao, frame vazio.
    frame_cmd_t pending;
    if (xQueueReceive(s_cmd_queue, &pending, 0) == pdTRUE)
        memcpy(tx_buf, &pending, sizeof(pending));

    spi_transaction_t t = {
        .length = 8 * SPI_XFER_BYTES,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err == ESP_OK)
        memcpy(telem_out, rx_buf, sizeof(*telem_out));

    return err;
}

// ── Task ──────────────────────────────────────────────────────────────────────

/* spi_master_task(): laco principal do ESP de comunicacao. Troca um frame por vez com o escravo; valida
 * cabecalho e CRC do frame_read_t recebido; se valido, imprime uma linha "TELEM,..." em CSV (estado x[],
 * medidas z[], duty por topologia d_bot[]/d_top[]) para ser capturada por um terminal serial/script de bancada.
 * O ritmo e adaptativo: se ha comando pendente na fila, o proximo ciclo roda IMEDIATAMENTE (para nao atrasar o
 * envio de comandos do operador); caso contrario, espera SPI_POLL_PERIOD_MS (100 ms) antes do proximo poll. */
void spi_master_task(void *arg)
{
    (void)arg;

    spi_master_init();

    while (1)
    {
        frame_read_t f;
        esp_err_t err = spi_exchange(&f);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_SPI, "SPI exchange failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(SPI_POLL_PERIOD_MS));
            continue;
        }

        // Valida e imprime a telemetria...
        if (f.header.magic != FRAME_MAGIC ||
            f.header.type != FRAME_TYPE_READ ||
            f.header.version != FRAME_VERSION)
        {
            ESP_LOGW(TAG_SPI, "Bad header: magic=0x%04X type=0x%02X ver=%u seq=%" PRIu32,
                     f.header.magic, f.header.type, f.header.version, f.header.seq);
        }
        else if (!read_frame_crc_ok(&f))
        {
            ESP_LOGW(TAG_SPI, "CRC mismatch (seq=%" PRIu32 ")", f.header.seq);
        }
        else
        {
            printf("TELEM,%" PRIu32, f.header.seq);
            for (int i = 0; i < SIG_COUNT; i++)
                printf(",%.4f", f.x[i]);
            for (int i = 0; i < Z_COUNT; i++)
                printf(",%.4f", f.z[i]);
            for (int i = 0; i < PHASES_COUNT; i++)
                printf(",%.4f", f.d_bot[i]);
            for (int i = 0; i < PHASES_COUNT; i++)
                printf(",%.4f", f.d_top[i]);
            printf("\n");
        }

        // Esvazia a fila rapido (roda de novo sem esperar); so pausa o periodo cheio quando ociosa.
        if (uxQueueMessagesWaiting(s_cmd_queue) == 0)
            vTaskDelay(pdMS_TO_TICKS(SPI_POLL_PERIOD_MS));
        // else: volta ao topo do laco imediatamente para enviar o proximo comando da fila
    }
}
