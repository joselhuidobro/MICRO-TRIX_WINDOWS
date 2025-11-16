#!/bin/bash
# Health check para Dockerfile

set -e

# Verificar Redis
redis-cli -u redis://:trixredis123@redis:6379/0 PING

# Verificar OPA
curl -f http://opa:8181/health > /dev/null

# Verificar LangGraph (si está activo)
if [ "$ENABLE_LANGGRAPH" = "true" ]; then
  curl -f http://langgraph-api:8000/health > /dev/null
fi

echo "✅ All systems operational"