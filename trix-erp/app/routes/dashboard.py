from flask import Blueprint, render_template, request, redirect, url_for, flash
from app.models import OrdenSemana
from decimal import Decimal
from datetime import datetime
from flask_login import login_required, current_user
import os
from app.scripts.decorators import nivel_requerido
from app.scripts.ventas.regresion_ventas_plot import plot_trend

from db2 import db

from app.models import OrdenSemana


import logging


logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()  
    ]
)

logger = logging.getLogger(__name__)
dashboard_bp = Blueprint('dashboard_bp', __name__)


@dashboard_bp.route('/dashboard', methods=['GET', 'POST'])
@login_required
@nivel_requerido(2,3) 
def dashboard():
        df_week = OrdenSemana.query.all()
        trend_plot =  plot_trend(df_week)   
        return render_template('dashboard_trix.html', trend_plot=trend_plot)


