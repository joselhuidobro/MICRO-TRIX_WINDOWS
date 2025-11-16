from flask import Blueprint, render_template, request, redirect, url_for, flash, jsonify, render_template_string, current_app
from app.models import Vecinos
from decimal import Decimal
from datetime import datetime
from db2 import db
from flask_login import login_required
import logging
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

departamento_bp = Blueprint('departamentos', __name__)

@departamento_bp.route('/get_vecinos/<int:depto_id>', methods=['GET'])
def get_vecinos(depto_id):
    vecinos = Vecinos.query.filter_by(id_depto=depto_id).all()
    vecinos_list = [{
        'id': vecino.id,
        'nombre': vecino.nombre,
        'correo': vecino.correo,
        'rfc': vecino.rfc
    } for vecino in vecinos]
    return jsonify(vecinos_list)


