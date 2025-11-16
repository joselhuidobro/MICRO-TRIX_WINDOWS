from langgraph.graph import StateGraph, END
from typing import TypedDict

# Estado: solo un campo de texto
class State(TypedDict):
    texto: str

def nodo_hello(state: State):
    """Nodo que transforma el texto a 'hello'"""
    return {"texto": "hello"}

# Construir grafo
workflow = StateGraph(State)
workflow.add_node("nodo_hello", nodo_hello)
workflow.add_edge("nodo_hello", END)
workflow.set_entry_point("nodo_hello")

# Compilar
app = workflow.compile()