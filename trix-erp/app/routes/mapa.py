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
mapa = Blueprint('mapa', __name__)


@mapa.route('/heat', methods=['GET', 'POST'])
@login_required
def heatmap_plot():
    logger.info("Antes de generar heat_plot")
    image = heat_plot()
    logger.info("Después de generar heat_plot")

    output_path = 'app/templates/map.html'  
 
    logger.info(f"Archivo map.html generado en: {output_path}")
    image.save(output_path)
    return render_template('heat.html', title='Heat Map')
