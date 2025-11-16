#!/usr/bin/env python3
import asyncio, json, logging, sys

# Logging con timestamps
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)

async def main():
    """Agente que procesa eventos Redis con LangGraph"""
    try:
        import redis.asyncio as redis
        from graph import app
        
        print("="*50)
        print("🐍 TRIX AGENT STARTING...")
        print("="*50)
        
        # Conexión a Redis con reintentos
        r = redis.Redis.from_url(
            "redis://:trixredis123@redis:6379/0",
            decode_responses=False,  # Importante: recibimos bytes
            retry_on_timeout=True
        )
        
        # Test de conexión
        pong = await r.ping()
        print(f"✅ [REDIS] Connected: {pong}")
        print(f"✅ [REDIS] Listening on queue: trix:events:queue")
        print("-"*50)
        
        while True:
            try:
                # Esperar evento (timeout de 5s para no saturar CPU)
                event = await r.brpop("trix:events:queue", timeout=5)
                
                if event:
                    # Evento es una tupla: (queue_name, data_bytes)
                    queue_name, data_bytes = event
                    print(f"\n📦 [EVENT] Raw: {event}")
                    
                    # Decodificar bytes a string
                    data_str = data_bytes.decode('utf-8')
                    print(f"📝 [EVENT] Decoded: {data_str}")
                    
                    # Parsear JSON
                    try:
                        data = json.loads(data_str)
                        print(f"🔍 [EVENT] Parsed: {data}")
                        
                        # EXTRAER EL NÚMERO (con validación)
                        numero = data.get("numero")
                        if numero is None:
                            print("❌ [ERROR] Campo 'numero' no encontrado en el evento")
                            continue
                        
                        print(f"🎯 [PROCESSING] Número: {numero}")
                        
                        # EJECUTAR GRAPHO
                        resultado = app.invoke({"mensaje": str(numero)})
                        print(f"✅ [RESULT] LangGraph output: {resultado}")
                        
                        # GUARDAR EN REDIS
                        key = f"resultado:{numero}"
                        await r.setex(key, 300, resultado["mensaje"])
                        print(f"💾 [REDIS] Saved '{key}' = {resultado['mensaje']}")
                        
                    except json.JSONDecodeError as e:
                        print(f"❌ [JSON ERROR] {e} - Data: '{data_str}'")
                        print("💡 Tip: Asegúrate de enviar JSON con comillas dobles: {\"numero\":7}")
                    
                else:
                    # Timeout, esperar de nuevo
                    print(".", end="", flush=True)
                    
            except Exception as e:
                print(f"\n❌ [LOOP ERROR] {type(e).__name__}: {e}")
                await asyncio.sleep(5)
                
    except Exception as e:
        print(f"\n💥 [FATAL ERROR] {type(e).__name__}: {e}")
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())