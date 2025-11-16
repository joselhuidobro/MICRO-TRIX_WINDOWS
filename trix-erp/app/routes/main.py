# routes.py

import logging
from flask import Blueprint, request, jsonify, render_template_string, redirect, url_for
from app.utils.prediction import predict_location
from app.utils.correo import correo
from app.utils.vecino_cercano import nearest_neighbor_tsp
from datetime import datetime
import pytz

from flask_cors import CORS
import psycopg2
from collections import defaultdict
import pandas as pd
from db import get_db_connection  # Importa la función de conexión desde db.py


main = Blueprint('main', __name__)

CORS(main, resources={r"/predict": {"origins": "https://automatizaciontrix.com.mx"}})  # Configuración de CORS
logging.basicConfig(level=logging.INFO)  # Configurar logging

def orden_zonas_clasificacion(df_agenda):
    # Determinar el orden de las zonas basado en la clasificación del primer cliente de la semana
        # Lista de zonas disponibles
        todas_las_zonas = ['TRIX_Norte', 'TRIX_Centro-sur', 'TRIX_Sur-este', 'Fuera_de_zona']
        if not df_agenda.empty:
            # Ordenar df_agenda por fecha si existe
            if 'fecha' in df_agenda.columns and df_agenda['fecha'].notnull().any():
                df_agenda = df_agenda.sort_values(by='fecha')
            primer_cliente = df_agenda.iloc[0]
            zona_primer_cliente = primer_cliente['clasificacion']
            logging.info(f"Zona del primer cliente: {zona_primer_cliente}")
            # Crear una lista ordenada de zonas empezando por la zona del primer cliente, si es fuera de zona  no debe ordenarlo

            if zona_primer_cliente in todas_las_zonas:
                indice_zona = todas_las_zonas.index(zona_primer_cliente)
                orden_zonas = todas_las_zonas[indice_zona:] + todas_las_zonas[:indice_zona]
            else:
                orden_zonas = todas_las_zonas
        else:
            orden_zonas = todas_las_zonas

        logging.info(f"Orden de zonas: {orden_zonas}")
        return(orden_zonas)

def borrar_agenda(cursor,conn):
        # Borrar datos existentes en la tabla 'agenda'
        cursor.execute("DELETE FROM agenda")
        conn.commit()
        logging.info("Datos antiguos borrados de la tabla 'agenda'.")

def guardar_nueva_agenda(df_final,cursor,conn):
        # Insertar la nueva agenda en la base de datos
        for index, row in df_final.iterrows():
            cursor.execute("""
                INSERT INTO agenda (dia, direccion, clasificacion, lat, lon, fecha, celular, horario)
                VALUES (%s, %s, %s, %s, %s, CURRENT_DATE, %s, %s)
            """, (
                row['dia'],
                row['direccion'],
                row['clasificacion'],
                row['lat'],
                row['lon'],
                row['celular'] if pd.notnull(row['celular']) else '0000000000',
                row['horario'] if pd.notnull(row['horario']) else '12:00:00'
            ))
        conn.commit()
        logging.info("Nueva agenda insertada correctamente en la tabla 'agenda'.")

        # Cerrar la conexión a la base de datos
        cursor.close()
        conn.close()

def asignar_dias(orden_zonas,clusters):

        # Asignar días a cada zona
        dias_semana = ['Lunes', 'Martes', 'Miércoles', 'Jueves', 'Viernes']
        agenda = []
        dia_index = 0

        for zona in orden_zonas:
            if zona in clusters:
                dia = dias_semana[dia_index % len(dias_semana)]  # Reutilizar días si es necesario
                for lugar in clusters[zona]:
                    lugar['dia'] = dia
                    agenda.append(lugar)
                dia_index += 1  # Incrementar el índice del día después de asignar una zona

        # Convertir la agenda a un DataFrame
        df_final = pd.DataFrame(agenda)
        logging.info(f"Agenda final: {df_final}")
        return(df_final)

def agenda_existente(conn,cursor): 
        # Obtener agenda existente de base de datos 
        cursor.execute("SELECT id, dia, direccion, clasificacion, lat, lon, fecha, celular, horario FROM agenda")
        resultados = cursor.fetchall()
        columnas = ['id', 'dia', 'direccion', 'clasificacion', 'lat', 'lon', 'fecha', 'celular', 'horario']
        df_existente = pd.DataFrame(resultados, columns=columnas)
        logging.info(f"Agenda existente: {df_existente}")
        return(df_existente)

@main.route('/')
def index():
    # Verificar si el usuario está autenticado
      return redirect(url_for('login.login')) 

# Ruta para manejar la predicción (POST)
@main.route('/predict', methods=['POST'])
def predict():
    if request.is_json:
        data = request.get_json()
        lat = data.get('lat')
        lon = data.get('lon')
        address = data.get('address')
        response_type = 'json'
    else:
        lat = request.form.get('lat')
        lon = request.form.get('lon')
        address = request.form.get('address')
        response_type = 'html'

    logging.info(f"Recibido lat: {lat}, lon: {lon}, address: {address}, (Tipo de dato: {response_type})")

    # Convertir lat y lon a flotantes y crear el objeto lugares
    try:
        lat = float(lat)
        lon = float(lon)
        address = str(address)
    except ValueError:
        return jsonify({'error': 'Latitud y longitud deben ser números'}), 400

    if lat is None or lon is None:
        error_message = 'Datos incompletos'
        logging.warning(error_message)
        if response_type == 'json':
            return jsonify({'error': error_message}), 400
        else:
            return render_template_string(f"<p>{error_message}</p>"), 400

    # Predicción de la clasificación y creación de la agenda
    try:
        conn = get_db_connection()         # Conectar a la base de datos
        cursor = conn.cursor()
        df_existente = agenda_existente(conn,cursor) # leer agenda existente en bd
        clasificacion = predict_location(lat, lon) #clasificar

        # si la clasificación es fuera de zona solo enviara la clasificación pero no hara nada en agenda
        if clasificacion != 'Fuera_de_zona':

            nuevo_lugar = {        # Crear DataFrame para el nuevo lugar
            'id': None,
            'dia': None,
            'direccion': address,
            'clasificacion': clasificacion,
            'lat': lat,
            'lon': lon,
            'fecha': None,
            'celular': None,
            'horario': None
                }
            df_nuevo = pd.DataFrame([nuevo_lugar])

            df_agenda = pd.concat([df_existente, df_nuevo], ignore_index=True) # Unir el nuevo lugar con el DataFrame existente , agenda sin ordenar

            # Agrupar los lugares por clasificación (zona)
            clusters = defaultdict(list)
            for index, row in df_agenda.iterrows():
              lugar = row.to_dict()
              clusters[lugar['clasificacion']].append(lugar)

        
            orden_zonas= orden_zonas_clasificacion (df_agenda) # nuevo orden con direccion nueva
            df_final = asignar_dias(orden_zonas,clusters)  # asignar dia a la lista ordenada
            borrar_agenda(cursor,conn) # borrar para actualizar
            guardar_nueva_agenda(df_final, cursor, conn) # guadar para actualizar

            correo(df_nuevo)# Llamar a la función correo

            # Obtener la fecha actual en la zona horaria de CDMX
            cdmx_tz = pytz.timezone('America/Mexico_City')
            fecha_cdmx = datetime.now(cdmx_tz)
            fecha_interes_datetime = fecha_cdmx.date()


            # Crear DataFrame a partir de df_final
            # data = pd.DataFrame(df_final, columns=['direccion', 'fecha', 'lat', 'lon', 'celular'])

            # Asegurar que la columna 'fecha' está en formato datetime
            # data['fecha'] = pd.to_datetime(data['fecha'])

# Asignar la fecha actual si 'fecha' está vacía o es nula
      #      if data['fecha'].isnull().all():
        #          data['fecha'] = fecha_interes_datetime

# Filtrar 'data' para que solo contenga las filas del día actual
       #     data = data[data['fecha'].dt.date == fecha_interes_datetime]
       #     data_filtrada = data[['fecha', 'lat', 'lon']] # Seleccionar las columnas necesarias
        #    data_filtrada = data_filtrada.dropna(subset=['lat', 'lon']) # Eliminar filas con valores nulos en 'lat' y 'lon'


            # Construir 'lista_puntos' directamente de 'data_filtrada'
          #  start_coords = (19.37321476729111, -99.15827983327816)
          #  lista_puntos = [start_coords] + list(zip(data_filtrada['lat'].astype(float), data_filtrada['lon'].astype(float)))
           # orden_indices = nearest_neighbor_tsp(lista_puntos) # Calcular la ruta utilizando el algoritmo del vecino más cercano
            #ruta_ordenada = [lista_puntos[i] for i in orden_indices] # Solo la ruta del día


           # logging.info(f"La ruta del día es: {ruta_ordenada}")

            


        else:
             pass
       
        # Enviar respuesta al front-end  , ojo la respuesta al front end si puede ser fuera de zona
        if response_type == 'json':
            return jsonify({'clasificacion': clasificacion}), 200
        else:
            result_html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <title>Resultado de la Predicción</title>
            </head>
            <body>
                <h1>Clasificación: {clasificacion}</h1>
                <a href="/predict">Hacer otra predicción</a>
            </body>
            </html>
            """
            return render_template_string(result_html)

    except Exception as e:
        logging.error(f"Error durante la predicción o generación de la agenda: {str(e)}")
        return jsonify({'error': str(e)}), 500
