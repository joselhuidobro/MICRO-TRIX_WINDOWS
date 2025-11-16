cmake_minimum_required(VERSION 3.16)

# Prioriza el directorio externo (micro-ROS) y luego tus components locales
set(EXTRA_COMPONENT_DIRS "/opt/extra_components;${CMAKE_SOURCE_DIR}/components")

# Configuración esencial
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp2024)

