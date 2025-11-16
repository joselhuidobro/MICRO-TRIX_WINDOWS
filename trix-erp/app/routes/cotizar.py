from flask import Blueprint, render_template, request, redirect, url_for, flash
from app.models import OrdenSemana
from decimal import Decimal
from datetime import datetime
from flask_login import login_required, current_user
import os
from app.scripts.decorators import nivel_requerido
from db2 import db
from app.models import cotizacion_historico
from app.scripts.cotizacion import cotizar_trix

import logging


logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()  
    ]
)

logger = logging.getLogger(__name__)
cotizar = Blueprint('cotizar', __name__)


@cotizar.route('/cotizacion', methods=['GET', 'POST'])
@login_required
@nivel_requerido(2,3) 
def cotizacion():
         data = cotizacion_historico.query.all()
         logger.info("Historico de cotizaciones: %s", data)
         
         return render_template('cotizacion.html', cotizaciones=data)


@cotizar.route('/generar_cotizacion', methods=['POST'])
@login_required
@nivel_requerido(2,3) 
def generar_cotizacion():
    form_data = request.form
    try:
        cotizar_trix(form_data)
        direccion = request.form.get('direccion')
        concepto = request.form.get('concepto')
        cel = request.form.get('cel')
        lat = request.form.get('lat')
        lon = request.form.get('lon')
        nombre = request.form.get('nombre')
        fecha = request.form.get('fecha')
        concepto_list = request.form.getlist('productos')  # Obtener la lista de productos

    except ValueError as ve:
            logger.error("Error en la conversión de valores: %s", ve)
            flash("Error en los valores numéricos del formulario. Por favor, verifica los datos.", "danger")
            return redirect(url_for('agenda.agenda_route'))
    if not all([cel, direccion, nombre]) or not concepto_list:
        flash('Por favor, completa todos los campos requeridos y selecciona al menos un producto.')
        return redirect(url_for('cotizar.generar_cotizacion'))
    
    concepto = ', '.join(concepto_list)
    # Crear un nuevo objeto OrdenSemana
    nueva_cotizacion = cotizacion_historico(
            cel=cel,
            nombre=nombre,
            fecha=fecha,
            direccion=direccion,
            concepto=concepto,
            lat=lat,
            lon=lon
        )
    # Guardar en la base de datos
    db.session.add(nueva_cotizacion)
    db.session.commit() 
    flash('Cotización creada exitosamente.')
    return redirect(url_for('cotizar.cotizacion'))  # Asumiendo que 'cotizacion' lista las cotizaciones

