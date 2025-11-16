from flask import Blueprint, render_template, request, redirect, url_for, flash, jsonify, render_template_string, current_app
from app.models import Building
from decimal import Decimal
from datetime import datetime
from db2 import db
from flask_login import login_required
import logging
from app.utils.prediction import predict_location
import os
from app.models import Departamento  # Asegúrate de importar tu modelo o ajustarlo a tu proyecto


# Configura el logger
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()  
    ]
)

logger = logging.getLogger(__name__)

building = Blueprint('building', __name__)

@building.route('/')
@login_required
def building_route():
    data = Building.query.all()
    return render_template('building.html', title="Building", Building=data)

@building.route('/building_register')
@login_required
def building_register():
    return render_template('building_register.html', title="Registrar Edificio")


@building.route('/building_save', methods=['POST'])
@login_required
def building_save():
 
    # Obtener datos del formulario
    edificio_lat = request.form.get('edificio_lat')
    edificio_lon = request.form.get('edificio_lon')
    predial = request.form.get('predial')
    fecha_construccion = request.form.get('fecha_construccion')
    cp = request.form.get('cp')
    no_deptos = request.form.get('no_deptos')
    no_pisos = request.form.get('no_pisos')
    accesos_peatonales = request.form.get('accesos_peatonales')
    accesos_vehiculares = request.form.get('accesos_vehiculares')
    lugares_estacionamiento = request.form.get('lugares_estacionamiento')


    # Crear una nueva instancia de Edificio
    nuevo_edificio = Building(
        lat=edificio_lat,
        lon=edificio_lon,
        predial=predial,
        fecha_construccion=int(fecha_construccion),
        cp=int(cp),
        no_deptos=int(no_deptos),
        no_pisos=int(no_pisos),
        accesos_peatonales=int(accesos_peatonales),
        accesos_vehiculares=int(accesos_vehiculares),
        lugares_estacionamiento=int(lugares_estacionamiento)
    )

    try:
        # Añadir y confirmar la nueva entrada en la base de datos
        db.session.add(nuevo_edificio)
        db.session.commit()
        flash('Edificio registrado exitosamente', 'success')
        return redirect(url_for('building.building_register'))
    except Exception as e:
        # Manejo de errores (puedes personalizar el mensaje)
        db.session.rollback()
        flash('Hubo un error al registrar el edificio. Por favor, intenta de nuevo.', 'danger')
        return redirect(url_for('building.building_register'))

@building.route('/get_departamentos/<int:building_id>', methods=['GET'])
def get_departamentos(building_id):
    departamentos = Departamento.query.filter_by(building_id=building_id).all()
    
    # Prepara la respuesta en formato JSON
    lista_deptos = []
    for d in departamentos:
        lista_deptos.append({
            "id": d.id,
            "no_del_depto": d.no_del_depto,
            "piso": d.piso,
            "cel": d.cel,
            "contacto": d.contacto,
            "rfc": d.rfc,
            "no_estacionamientos": d.no_estacionamientos,
            "roof_garden": d.roof_garden,
            "puerta_automatica": d.puerta_automatica,
            "no_cuartos": d.no_cuartos
            
        })
    return jsonify(lista_deptos)
