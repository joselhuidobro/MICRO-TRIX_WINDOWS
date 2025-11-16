#!/usr/bin/env python3
from graph import app

# Estado inicial vacío
estado_inicial = {"mensaje": ""}

# Ejecutar grafo
print("🚀 Ejecutando grafo Step 2...\n")
resultado = app.invoke(estado_inicial)

# Mostrar resultado final
print(f"\n✅ Resultado final: '{resultado['mensaje']}'")