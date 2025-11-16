
import pandas as pd
import matplotlib
import seaborn as sns
import pytz
import folium
import networkx as nx
from itertools import combinations
from haversine import haversine, Unit
from app.scripts.vecino_cercano import draw_route
import os
from flask import Blueprint, render_template, redirect, url_for, request ,flash, jsonify# Blueprint : trae importaciones para no hacerlas circulares
from folium.plugins import HeatMap
from PIL import Image
from datetime import datetime
from folium.plugins import HeatMap
from app.scripts.vecino_cercano import draw_route, nearest_neighbor_tsp
from db2 import db
from app.models import OrdenSemana
import logging


dbs_2 = Blueprint('dbs_2',__name__)


# Configura el logger
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()  
    ]
)

logger = logging.getLogger(__name__)


matplotlib.use('agg')

from geopy.geocoders import Nominatim
import folium

def generate_popup_content(direccion):
    return f"""
        <div style="width: 300px">
            <h3>Dirección: {direccion}</h3>
            <p>Motor: </p>
            <h4>Ver deptos</h4>
           <h4>Ver controladores</h4>
       </div>
    """

def generate_popup_content_competencia(direccion,nom):
    return f"""
        <div style="width: 300px">
            <h3>Dirección: {direccion}</h3>
            <p>Competencia: {nom}</p>
         
       </div>
    """
    

def heat_plot():
    # Zona horaria de CDMX y fecha actual
    cdmx_tz = pytz.timezone('America/Mexico_City')
    fecha_utc = datetime.utcnow()
    fecha_cdmx = fecha_utc.replace(tzinfo=pytz.utc).astimezone(cdmx_tz)
    fecha_interes_datetime = fecha_cdmx.date()

    # Consultar todos los registros de la base de datos
    registros = OrdenSemana.query.all()

    # Convertir los registros en un DataFrame
    data = [
        {
            "id": registro.id,
            "cel": registro.cel,
            "nombre": registro.nombre,
            "fecha": registro.fecha,
            "direccion": registro.direccion,
            "concepto": registro.concepto,
            "categoria": registro.categoria,
            "cat": registro.estado,      # Mapear la categoría desde el estado o algún campo similar
            "lat": registro.lat,
            "lon": registro.lon,
            "en_ruta":registro.en_ruta
        }
        for registro in registros
    ]
    df = pd.DataFrame(data)
    logging.info(f"df de la semana: {df}")


    #filtrar los que no tengan lat y lon



        # Filtrar por en_ruta == True y eliminar valores nulos en lat, lon
    if not df.empty:
        df['fecha'] = pd.to_datetime(df['fecha'])
    
    # Filtrar únicamente aquellos que tengan en_ruta = True
        data_filtrada = df[
          (df['en_ruta'] == True) &
          (~df['lat'].isna()) &
          (~df['lon'].isna())
         ]
    
    # Seleccionar columnas necesarias
        data_filtrada = data_filtrada[['fecha', 'lat', 'lon', 'direccion', 'cat', 'nombre']]
    else:
        data_filtrada = pd.DataFrame(columns=['fecha', 'lat', 'lon', 'direccion', 'cat', 'nombre'])

    start_coords = (19.37321476729111, -99.15827983327816)
    lista_puntos = [start_coords]

    for index, row in data_filtrada.iterrows():
        lat = float(row['lat'])
        lon = float(row['lon'])
        lista_puntos.append((lat, lon))

    # Verificar la lista de puntos
    orden_indices = nearest_neighbor_tsp(lista_puntos)  # organiza direcciones

    lista_completa_ordenada = [lista_puntos[i] for i in orden_indices]  # Reorganiza rutas según orden_indices



    logging.info(f"lista ordenada con direcciones: {lista_completa_ordenada}")

 
       
    folium_map = folium.Map(location=start_coords, zoom_start=14)
    
    # si hay cambios en point_list refrescar ruta /heat
    folium_map = draw_route(folium_map, lista_completa_ordenada)
    folium_map.save("tsp_route.html")

    grupo = folium.FeatureGroup(name='TRIX')

    folium_map.add_child(grupo)
    return folium_map
 