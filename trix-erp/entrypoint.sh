#!/usr/bin/env bash
set -e

PORT=${PORT:-6000}
echo "Puerto asignado: $PORT"

# Cargar /run/stack.env de forma segura
if [ -f /run/stack.env ]; then
  set -o allexport
  # shellcheck disable=SC1091
  . /run/stack.env
  set +o allexport
fi

echo "ENV DEBUG: DATABASE_URL=${DATABASE_URL:-<empty>} DB_USER=${DB_USER:-<empty>} DB_NAME=${DB_NAME:-<empty>}"

exec gunicorn "run:app" \
  --bind "0.0.0.0:${PORT:-6000}" \
  --workers "${WORKERS:-2}" \
  --threads "${THREADS:-4}" \
  --timeout "${TIMEOUT:-120}"

