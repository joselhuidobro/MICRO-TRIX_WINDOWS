idf_component_register(
    SRCS
        "main.cpp"
	"trix_main_loop.cpp"
  	"encoder.cpp"
    "motor.cpp"    
    INCLUDE_DIRS
        "."
    REQUIRES               
        micro_ros_espidf_component
        uros_helper
        freertos
        driver
        esp_timer
        pid_ctrl
        i2c_bus
        mpu6050
        nvs_flash

    PRIV_REQUIRES           

        trix_wifi_connection
        trix_kalman_filter
        trix_autotune
        trix_udp_telemetria
        trix_esc
        trix_imu
        trix_PID
)

# Forzar estándar de C++ si lo necesitas
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Definiciones opcionales para habilitar/inhabilitar bloques de código en main.cpp
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    USE_MICROROS=1
    ENABLE_TRIX_AUTOTUNE=1
)

set(EXTRA_COMPONENT_DIRS "/opt/extra_components;${CMAKE_SOURCE_DIR}/components")

