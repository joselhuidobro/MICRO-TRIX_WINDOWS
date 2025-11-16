from flask import Blueprint, render_template, request, redirect, url_for, flash
from app.models import OrdenSemana
from decimal import Decimal
from datetime import datetime
from flask_login import login_required, current_user
import os
from app.scripts.decorators import nivel_requerido
from db2 import db

import logging


logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()  
    ]
)

logger = logging.getLogger(__name__)
inmobiliaria_bp = Blueprint('inmobiliaria_bp', __name__)


@inmobiliaria_bp.route('/inmobiliaria', methods=['GET', 'POST'])
@login_required
@nivel_requerido(2,3) 
def inmobiliaria():
         logger.info("Historico de cotizaciones: %s")
         
         return render_template('cotizacion.html')


