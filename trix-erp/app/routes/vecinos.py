from flask import Blueprint, render_template, request, redirect, url_for, flash, jsonify, render_template_string, current_app
from app.models import User, Departamento, Vecinos
from decimal import Decimal
from datetime import datetime
from db2 import db
from flask_login import current_user, login_required
import logging
from app.scripts.decorators import nivel_requerido

# Configura el logger
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)

vecinos = Blueprint('vecinos', __name__)

@vecinos.route('/')
@nivel_requerido(1,2,3)
@login_required
def vecinos_route():
    try:
        # Obtener todos los vecinos
        vecinos_all = Vecinos.query.all()

        # Obtener el correo del usuario actual
        current_correo = current_user.correo

        # Comparar el correo del usuario actual con el correo de cada vecino
        for vecino in vecinos_all:
            if current_correo == vecino.correo:
                logging.info(f'El usuario actual ({current_correo}) coincide con el vecino {vecino.nombre} (ID: {vecino.id})')
            else:
                logging.info(f'El usuario actual ({current_correo}) no coincide con el vecino {vecino.nombre} (ID: {vecino.id})')

    except Exception as e:
        logging.error(f'Error al comparar datos: {e}')
    
    # Asumiendo que 'user' se obtiene de alguna parte
    user = User.query.get(current_user.id)  
    return render_template('vecinos.html', title="Vecinos", user=user, vecinos=vecinos_all)