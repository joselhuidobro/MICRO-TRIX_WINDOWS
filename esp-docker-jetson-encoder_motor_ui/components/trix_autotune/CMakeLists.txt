idf_component_register(
    SRCS "trix_autotune.c"
    INCLUDE_DIRS "include"
    REQUIRES trix_PID trix_kalman_filter trix_esc esp_timer
)

