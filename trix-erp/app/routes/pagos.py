from flask import Blueprint, render_template, request, redirect, url_for, flash
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
pago = Blueprint('pago', __name__)


@pago.route('/')
@login_required
def pago_route():
    #data = Pago.query.all()
    return render_template('add_pago.html', title="Pago")
