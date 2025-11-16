# db2.py

import os
from flask_sqlalchemy import SQLAlchemy

db = SQLAlchemy()

def get_database_uri():
    # Primero, intenta obtener DATABASE_URL
    database_url = os.environ.get('DATABASE_URL')
    if database_url:
        # Corregir el prefijo si es necesario
        if database_url.startswith('postgres://'):
            database_url = database_url.replace('postgres://', 'postgresql://', 1)
        return database_url
    else:
        # Si DATABASE_URL no está definida, construye la URI manualmente
        db_user = os.environ.get('DB_USER')
        db_password = os.environ.get('DB_PASSWORD')
        db_host = os.environ.get('DB_HOST', 'localhost')
        db_port = os.environ.get('DB_PORT', '5432')
        db_name = os.environ.get('DB_NAME')

        if not all([db_user, db_password, db_name]):
            raise ValueError("Faltan variables de entorno para la configuración de la base de datos.")

        return f'postgresql://{db_user}:{db_password}@{db_host}:{db_port}/{db_name}'
