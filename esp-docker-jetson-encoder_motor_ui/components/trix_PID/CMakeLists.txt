idf_component_register(
    SRCS "trix_PID.c"
    INCLUDE_DIRS "include"
    REQUIRES  pid_ctrl        # <-- aquí la magia

)

