from graph import app

# Estado inicial
estado_inicial = {"texto": "world"}

# Ejecutar grafo
print("🚀 Ejecutando grafo...")
resultado = app.invoke(estado_inicial)

# Resultado
print(f"✅ Resultado: {resultado}")
# Debe mostrar: {'texto': 'hello'}