from flask import Blueprint, render_template, request, redirect, url_for, flash, jsonify, render_template_string, current_app
from app.models import OrdenSemana
from decimal import Decimal
from datetime import datetime
from flask_login import login_required
import logging
import os
from app.scripts.plotdata_heatmap import heat_plot


# Configura el logger
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler()  
    ]
)

logger = logging.getLogger(__name__)
map_box = Blueprint('mapbox', __name__)


@map_box.route('/', methods=['GET', 'POST'])
@login_required
def mapa_box():
    return render_template('mapbox.html')