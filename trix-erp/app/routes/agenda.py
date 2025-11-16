from flask import Blueprint, render_template, request, redirect, url_for, flash, jsonify, render_template_string, current_app
from app.models import OrdenSemana
from app.models import Historico_ventas
from decimal import Decimal
from datetime import datetime
from db2 import db
from flask_login import login_required
import logging
from app.utils.prediction import predict_location
from app.scripts.decorators import nivel_requerido
import os

# Configura el logger
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()  
    ]
)

logger = logging.getLogger(__name__)
agenda = Blueprint('agenda', __name__)

@agenda.route('/')
@nivel_requerido(2,3) 
@login_required
def agenda_route():
    """Renderiza la página principal de agenda con las órdenes de la semana."""
    data = OrdenSemana.query.all()
    return render_template('agenda.html', title="Agenda", semanas=data)

@agenda.route('/get_google_api_key', methods=['GET'])
@nivel_requerido(2,3) 
@login_required
def get_google_api_key():
    api_key = current_app.config.get('GOOGLE_API_KEY')
    if not api_key:
        return jsonify({'error': 'Google API Key not configured'}), 500
    return jsonify({'api_key': api_key})



@agenda.route('/agregar_orden', methods=['POST'])
@nivel_requerido(2,3) 
@login_required
def agregar_orden():
    google_api_key = current_app.config['GOOGLE_API_KEY']
    logger.debug(f"GOOGLE_API_KEY desde config: {current_app.config.get('GOOGLE_API_KEY')}")
    """Maneja la creación de una nueva orden."""
    logger.debug("Datos recibidos en la solicitud POST: %s", request.form.to_dict())
    try:
        # Validación y obtención de datos
        cel = request.form.get('cel')
        nombre = request.form.get('nombre')
        fecha = request.form.get('fecha')

        if not cel or not nombre or not fecha:
            flash("Faltan datos obligatorios: Cel, Nombre o Fecha.", "danger")
            return redirect(url_for('agenda.agenda_route'))

        try:
            fecha = datetime.strptime(fecha, '%Y-%m-%d')
        except ValueError:
            logger.error("Formato de fecha inválido.")
            flash("Fecha inválida. Por favor, ingresa una fecha en el formato YYYY-MM-DD.", "danger")
            return redirect(url_for('agenda.agenda_route'))

        direccion = request.form.get('direccion')
        concepto = request.form.get('concepto')
        categoria = request.form.get('categoria')
        forma_de_pago = request.form.get('forma_de_pago')
        estado = request.form.get('pendiente')
        dia_sem = request.form.get('dia_sem')

        try:
            no_semana = int(request.form.get('no_semana', 0))
            cantidad = Decimal(request.form.get('cantidad', '0'))
            precio_unitario = Decimal(request.form.get('precio_unitario', '0'))
            total_cotizado = Decimal(request.form.get('total_cotizado', '0'))
            total_pagado = Decimal(request.form.get('total_pagado', '0'))
            latitud = float(request.form.get('lat', '0'))
            longitud = float(request.form.get('lon', '0'))
        except ValueError as ve:
            logger.error("Error en la conversión de valores: %s", ve)
            flash("Error en los valores numéricos del formulario. Por favor, verifica los datos.", "danger")
            return redirect(url_for('agenda.agenda_route'))

        # Crear un nuevo objeto OrdenSemana
        nueva_orden = OrdenSemana(
            cel=cel,
            nombre=nombre,
            fecha=fecha,
            direccion=direccion,
            concepto=concepto,
            categoria=categoria,
            no_semana=no_semana,
            dia_semana=dia_sem,
            cantidad=cantidad,
            precio_unitario=precio_unitario,
            total_cotizado=total_cotizado,
            total_pagado=total_pagado,
            forma_de_pago=forma_de_pago,
            estado=estado,
            lat=latitud,
            lon=longitud
        )


        # Guardar en la base de datos
        db.session.add(nueva_orden)
        db.session.commit()

        flash("Orden guardada con éxito en la base de datos.", "success")
        return redirect(url_for('agenda.agenda_route'))

    except Exception as ex:
        db.session.rollback()
        logger.error("Error al guardar la orden: %s", ex)
        flash(f"Error al guardar la orden: {ex}", "danger")
        return redirect(url_for('agenda.agenda_route'))

@agenda.route('/update_status', methods=['POST'])
@nivel_requerido(2,3) 
@login_required
def update_status():
    """Actualiza el estado de una orden."""
    
    try:
        data = request.get_json()
        logger.debug("Datos recibidos para actualización de estado: %s", data)

        if not data or 'id' not in data or 'status' not in data:
            logger.error("Solicitud inválida. Datos: %s", data)
            return {"error": "Bad Request - Missing 'id' or 'status'"}, 400

        # Obtener datos de la solicitud
        order_id = data['id']
        new_status = data['status']

        # Buscar la orden en la base de datos
        orden = OrdenSemana.query.get(order_id)
        if not orden:
            logger.error("Orden con ID %s no encontrada.", order_id)
            return {"error": f"Order with ID {order_id} not found."}, 404

        # Actualizar el estado
        orden.estado = new_status
        db.session.commit()

        logger.info("Estado de la orden ID %s actualizado a: %s", order_id, new_status)
        return {"message": "Status updated successfully"}, 200

    except Exception as ex:
        logger.error("Error al actualizar el estado: %s", ex)
        db.session.rollback()
        return {"error": f"Internal Server Error: {ex}"}, 500

    


@agenda.route('/update_time', methods=['POST'])
@nivel_requerido(2,3) 
@login_required
def update_time():
    data = request.get_json()  # Obteniendo los datos de la petición POST
    if not data or 'time' not in data or 'id' not in data:
        return "Bad Request", 400  # Si no hay datos o no contiene 'time' o 'id', devuelve un error

    hora_inicio = data['time']  # Obteniendo el valor de la hora
    id = data['id']  # Obteniendo el identificador

@agenda.route('/borrar_venta/<int:id>', methods=['POST'])
@nivel_requerido(2,3) 
@login_required
def borrar_venta(id):
    try:
        orden = OrdenSemana.query.get(id)
        if not orden:
            flash(f"No se encontró la orden con id={id}.", "danger")
            return redirect(url_for('agenda.agenda_route'))

        # Borra el registro de la base de datos
        db.session.delete(orden)
        db.session.commit()

        flash(f"La orden con id={id} ha sido eliminada correctamente.", "success")
        return redirect(url_for('agenda.agenda_route'))
    except Exception as ex:
        db.session.rollback()
        logger.error(f"Error al borrar la orden con id={id}: {ex}")
        flash(f"Error al borrar la orden: {ex}", "danger")
        return redirect(url_for('agenda.agenda_route'))

@agenda.route('/get_orden/<int:id>', methods=['GET'])
@nivel_requerido(2,3) 
@login_required
def get_orden(id):
    """Obtiene los detalles de una orden para renderizar el formulario de edición."""
    try:
        # Busca la orden por ID en la base de datos
        orden = OrdenSemana.query.get(id)

        if not orden:
            flash(f"No se encontró la orden con ID={id}.", "danger")
            return redirect(url_for('agenda.agenda_route'))

        # Renderiza el formulario de edición con los datos de la orden
        return render_template('edit_agenda.html', orden=orden)

    except Exception as ex:
        logger.error(f"Error al obtener la orden con ID={id}: {ex}")
        flash(f"Error al procesar la orden: {ex}", "danger")
        return redirect(url_for('agenda.agenda_route'))
    

@agenda.route('/actualizar_v/<int:id>', methods=['POST'])
@nivel_requerido(2,3) 
@login_required
def actualizar_semana(id):
    """Actualiza los datos de una venta específica según su ID."""
    try:
        # Busca la orden en la base de datos
        orden = OrdenSemana.query.get(id)
        if not orden:
            flash(f"No se encontró la orden con ID={id}.", "danger")
            return redirect(url_for('agenda.agenda_route'))
        
        # Obtiene los datos del formulario
        orden.cel = request.form.get('cel', orden.cel)
        orden.nombre = request.form.get('nombre', orden.nombre)
        orden.fecha = request.form.get('fecha', orden.fecha)
        orden.direccion = request.form.get('direccion', orden.direccion)
        orden.concepto = request.form.get('concepto', orden.concepto)
        orden.categoria = request.form.get('categoria', orden.categoria)
        orden.no_semana = request.form.get('no_semana', orden.no_semana)
        orden.dia_semana = request.form.get('dia_semana', orden.dia_semana)
        orden.cantidad = Decimal(request.form.get('cantidad', orden.cantidad or '0'))
        orden.precio_unitario = Decimal(request.form.get('precio_unitario', orden.precio_unitario or '0'))
        orden.total_cotizado = Decimal(request.form.get('total_cotizado', orden.total_cotizado or '0'))
        orden.total_pagado = Decimal(request.form.get('total_pagado', orden.total_pagado or '0'))
        orden.forma_de_pago = request.form.get('forma_de_pago', orden.forma_de_pago)
        orden.estado = request.form.get('estado', orden.estado)
        orden.lat = float(request.form.get('lat', orden.lat or '0'))
        orden.lon = float(request.form.get('lon', orden.lon or '0'))
    
        # Guarda los cambios en la base de datos
        db.session.commit()
        flash(f"La orden con ID={id} ha sido actualizada correctamente.", "success")
    except Exception as ex:
        db.session.rollback()
        logger.error(f"Error al actualizar la orden con ID={id}: {ex}")
        flash(f"Error al actualizar la orden: {ex}", "danger")
    
    return redirect(url_for('agenda.agenda_route'))

# Ruta para manejar la predicción (POST)
@agenda.route('/predict', methods=['POST'])
@nivel_requerido(2,3) 
@login_required
def predict():
    if request.is_json:
        data = request.get_json()
        lat = data.get('lat')
        lon = data.get('lon')
        address = data.get('dir')
        response_type = 'json'
    else:
        lat = request.form.get('lat')
        lon = request.form.get('lon')
        address = request.form.get('address')
        response_type = 'html'

    logging.info(f"Recibido lat: {lat}, lon: {lon}, address: {address}, (Tipo de dato: {response_type})")

    try:
        lat = float(lat)
        lon = float(lon)
        address = str(address)
    except ValueError:
        error_message = 'Latitud y longitud deben ser números'
        logging.warning(error_message)
        if response_type == 'json':
            return jsonify({'error': error_message}), 400
        else:
            return render_template_string(f"<p>{error_message}</p>"), 400

    if lat is None or lon is None:
        error_message = 'Datos incompletos'
        logging.warning(error_message)
        if response_type == 'json':
            return jsonify({'error': error_message}), 400
        else:
            return render_template_string(f"<p>{error_message}</p>"), 400

    # Clasificar la ubicación
    clasificacion = predict_location(lat, lon)
    logging.info(f"La zona es : {clasificacion}")

    # Retornar la respuesta según el tipo solicitado
    if response_type == 'json':
        return jsonify({'clasificacion': clasificacion}), 200
    else:
        return render_template_string(f"<p>La clasificación de la ubicación es: {clasificacion}</p>")


    
@agenda.route('/update_ruta_ajax', methods=['POST'])
@nivel_requerido(2,3) 
@login_required
def update_ruta_ajax():
    data = request.get_json()
    if not data:
        return jsonify({"error": "No se recibió JSON"}), 400

    orden_id = data.get('id')
    en_ruta = data.get('en_ruta')

    if orden_id is None or en_ruta is None:
        return jsonify({"error": "Faltan parámetros"}), 400

    try:
        orden = OrdenSemana.query.get_or_404(orden_id)
        orden.en_ruta = en_ruta
        db.session.commit()
        return jsonify({
            "message": "Estado 'en_ruta' actualizado correctamente",
            "orden_id": orden_id,
            "en_ruta": en_ruta
        }), 200
    except Exception as e:
        db.session.rollback()
        return jsonify({"error": str(e)}), 500
    





@agenda.route('/actualizar_tabla', methods=['POST'])
@login_required
def actualizar_tabla():
    try:
        # Obtener todos los registros de OrdenSemana
        ordenes = OrdenSemana.query.all()
        
        if not ordenes:
            return jsonify({'mensaje': 'No hay órdenes para actualizar.'}), 200

        # Iterar sobre cada orden y crear una nueva entrada en HistoricoVentas
        for orden in ordenes:
            historico = Historico_ventas(
                cel=orden.cel,
                nombre=orden.nombre,
                fecha=orden.fecha,
                direccion=orden.direccion,
                concepto=orden.concepto,
                categoria=orden.categoria,
                no_semana=orden.no_semana,
                dia_semana=orden.dia_semana,
                cantidad=orden.cantidad,
                precio_unitario=orden.precio_unitario,
                total_cotizado=orden.total_cotizado,
                total_pagado=orden.total_pagado,
                forma_de_pago=orden.forma_de_pago,
                estado=orden.estado,
                lat=orden.lat,
                lon=orden.lon
            )
            db.session.add(historico)
       
        db.session.commit()
        
        return jsonify({'mensaje': 'Órdenes actualizadas correctamente en el historial y ordenes actuales limpiadas.'}), 200

    except Exception as e:
        db.session.rollback()
        return jsonify({'mensaje': 'Ocurrió un error al actualizar las órdenes.', 'error': str(e)}), 500



