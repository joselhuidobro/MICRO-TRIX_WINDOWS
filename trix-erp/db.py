# db.py
import os
import psycopg2

def get_db_connection():
    database_url = os.environ['DATABASE_URL']
    conn = psycopg2.connect(database_url, sslmode='require')
    return conn
