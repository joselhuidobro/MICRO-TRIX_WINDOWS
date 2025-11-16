set(SRC_FILES "src/i2c_bus.c")

if(CONFIG_I2C_BUS_SUPPORT_SOFTWARE)
    list(APPEND SRC_FILES "src/i2c_bus_soft.c")
endif()

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES driver
)

