#!/bin/bash
set -e

# Activa las variables de entorno y herramientas del ESP-IDF.
# Esto es necesario para poder usar el comando idf.py. [4, 2]
source /opt/esp/idf/export.sh

# Ejecuta cualquier comando que se pase a 'docker run' (por defecto, "bash").
exec "$@"
