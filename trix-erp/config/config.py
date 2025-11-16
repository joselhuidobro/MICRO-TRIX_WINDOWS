
from flask import current_app


import os

class Config:
    GOOGLE_API_KEY = os.getenv('GOOGLE_API_KEY')
    SECRET_KEY = os.getenv('SECRET_KEY', 'valor_predeterminado_seguro')


def get_google_api_key():
    """Función de utilidad para obtener la clave de Google."""
    from flask import current_app
    return current_app.config.get('GOOGLE_API_KEY', 'default_key')