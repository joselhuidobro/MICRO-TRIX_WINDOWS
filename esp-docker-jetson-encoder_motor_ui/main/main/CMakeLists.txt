set(SOURCES
    "main.cpp" 
)

file(GLOB_RECURSE EXTRA_COMPONENTS "PID_v1/*.cpp")
list(APPEND SOURCES "${EXTRA_COMPONENTS}")


idf_component_register(SRCS "${SOURCES}"
                       INCLUDE_DIRS "." "PID_v1")






#include "driver/ledc.h"