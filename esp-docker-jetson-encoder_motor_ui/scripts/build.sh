#!/usr/bin/env bash
# --- build.sh ---------------------------------------------------------
set -euo pipefail
source /opt/esp-idf/export.sh

cd /esp2024

# opcional: fullclean
if [[ ${1-} == clean ]]; then
  echo -e "\n🧹  idf.py fullclean\n"
  idf.py fullclean
fi

printf "\n🔧  ESP-IDF %s  •  Target: %s\n" \
       "$(idf.py --version | head -1)" \
       "$(idf.py show-target)"

echo -e "\n🏗  Iniciando build...\n"
idf.py build

BIN=$(find build -name '*.bin' -type f | head -1)
SIZE=$(stat -c%s "$BIN")
printf "\n✅  Build finalizado  |  📦  %s  (%0.1f KB)\n" "$BIN" "$(echo "$SIZE/1024" | bc -l)"

