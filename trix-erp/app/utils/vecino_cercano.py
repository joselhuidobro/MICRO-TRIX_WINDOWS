from haversine import haversine, Unit
from itertools import combinations
from folium.plugins import HeatMap
import folium
import networkx as nx



def nearest_neighbor_tsp(point_list):
    
    G = nx.Graph()# Crear un grafo vacío
    for i, point in enumerate(point_list): # Añadir nodos al grafo, cada nodo representa un punto en la lista de puntos
        G.add_node(i, pos=point)

    # Añadir bordes con peso basado en la distancia entre puntos
    for pair in combinations(range(len(point_list)), 2): # Utilizamos la función haversine para calcular la distancia en metros entre dos puntos
        p1, p2 = pair
        pos1 = G.nodes[p1]['pos']
        pos2 = G.nodes[p2]['pos']
        distance = haversine(pos1, pos2, unit=Unit.METERS)
        G.add_edge(p1, p2, weight=distance)

    # Inicializar variables para la heurística del vecino más cercano
    start_node = 0
    visited = set()
    visited.add(start_node)
    current_node = start_node
    path = [start_node]

    # Continuar hasta que todos los nodos hayan sido visitados
    while len(visited) < len(point_list):
        # Inicializar variables para encontrar el vecino más cercano
        min_dist = float('inf')
        next_node = None

        # Buscar el vecino más cercano que aún no haya sido visitado
        for neighbor in G[current_node]:
            if neighbor not in visited:
                distance = G[current_node][neighbor]['weight']
                if distance < min_dist:
                    min_dist = distance
                    next_node = neighbor

        # Agregar el vecino más cercano al conjunto de nodos visitados y a la ruta
        visited.add(next_node)
        path.append(next_node)
        current_node = next_node

    
    path.append(start_node) # Agregar el nodo de inicio al final de la ruta para completar el ciclo

    return path


def draw_route(folium_map, point_list):
    # Dibujar la ruta en el mapa de folium
    # Iterar a través de los pares de nodos adyacentes en la ruta
    
    point_list = [[float(coord[0]), float(coord[1])] for coord in point_list] # Convertir las coordenadas de strings a float
 
    for i in range(len(point_list) - 1):
        # Obtener las coordenadas de los dos nodos adyacentes
        current_point = point_list[i]
        next_point = point_list[i + 1]

        # Dibujar una línea entre los dos nodos adyacentes en el mapa de folium
        folium.PolyLine(locations=[current_point, next_point], weight=5).add_to(folium_map)

    return folium_map