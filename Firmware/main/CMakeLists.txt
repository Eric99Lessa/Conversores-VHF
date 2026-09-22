idf_component_register(
    SRCS "uart_task.c" "optim.c" "model.c" "observer.c" "adc.c" "communication.c" "control.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)