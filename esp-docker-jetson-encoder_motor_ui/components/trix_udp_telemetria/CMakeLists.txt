idf_component_register(
    SRCS         "trix_udp_telemetria.c"
    INCLUDE_DIRS "include"          # ← antes decía "."
    REQUIRES     lwip
)

