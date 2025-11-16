from langgraph.graph import StateGraph, END
from typing import TypedDict

class State(TypedDict):
    mensaje: str

def nodo_hello(state: State):
    print("📦 Nodo 'hello': Creando mensaje inicial")
    return {"mensaje": "hello"}

def nodo_incrementar(state: State):
    print(f"📝 Nodo 'incrementar': Concatenando a '{state['mensaje']}'")
    return {"mensaje": state["mensaje"] + " world"}

# Construir grafo
workflow = StateGraph(State)
workflow.add_node("hello", nodo_hello)
workflow.add_node("incrementar", nodo_incrementar)

# Conectar: hello → incrementar → FIN
workflow.add_edge("hello", "incrementar")
workflow.add_edge("incrementar", END)

workflow.set_entry_point("hello")
app = workflow.compile()