
# cotizaciones_hist.py

from db2 import db
from datetime import datetime

# Modelo para la tabla OrdenesSemana
class cotizacion_historico(db.Model):
    __tablename__ = 'cotizaciones_historico'

    id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    fecha = db.Column(db.Date, nullable=False, default=datetime.utcnow)
    cel = db.Column(db.String(20), nullable=False)
    lat = db.Column(db.Float)
    lon = db.Column(db.Float)
    direccion = db.Column(db.String(255))
    nombre = db.Column(db.String(100), nullable=False)
    concepto = db.Column(db.String(100))



    def __init__(self, cel, nombre, fecha, direccion=None, concepto=None, lat=None, lon=None):
        self.cel = cel
        self.nombre = nombre
        self.fecha = fecha
        self.direccion = direccion
        self.concepto = concepto
        self.lat = lat
        self.lon = lon




    def __repr__(self):
        return f"<cotizacion_historico(id={self.id}, nombre='{self.nombre}', fecha={self.fecha})>"