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
    // ── Communication ESP ──────────────────────────────────────────────────
    xTaskCreatePinnedToCore(spi_master_task, "spi_task", 4096, NULL, PRIO_HIGH, NULL, CORE_MAIN);
    xTaskCreatePinnedToCore(uart_task, "uart_task", 4096, NULL, PRIO_LOW, NULL, CORE_SCND);
#else
    // ── Control ESP ──────────────────────────────────────────────────
    static task_params_t params = {
        .telem_mux = portMUX_INITIALIZER_UNLOCKED,
        .pwm_mode = PWM_MODE_BOOST,
        .ctrl_mode = CTRL_OPEN,
    };

    xTaskCreatePinnedToCore(control_task, "ctrl", 4096, &params, PRIO_HIGH, NULL, CORE_MAIN);
    xTaskCreatePinnedToCore(spi_slave_task, "spi_task", 4096, &params, PRIO_LOW, NULL, CORE_SCND);
#endif
}