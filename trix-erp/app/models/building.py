# cotizaciones_hist.py

from db2 import db
from datetime import datetime

class Building(db.Model):
    __tablename__ = 'buildings'

    id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    fecha = db.Column(db.Date, nullable=False, default=datetime.utcnow)  # Fecha de registro en la base
    cel = db.Column(db.String(20), nullable=False)
    
    # Nuevos campos solicitados
    rfc = db.Column(db.String(20))
    administrador = db.Column(db.String(100), nullable=False)
    num_predial = db.Column(db.Integer)
    fecha_construccion = db.Column(db.Integer)  # Asumimos año de construcción como entero
    cp = db.Column(db.Integer)
    no_deptos = db.Column(db.Integer)
    no_pisos = db.Column(db.Integer)
    accesos_peatonales = db.Column(db.Integer)
    accesos_vehiculares = db.Column(db.Integer)
    lugares_estacionamiento = db.Column(db.Integer)
    
    lat = db.Column(db.Float)
    lon = db.Column(db.Float)
    direccion = db.Column(db.String(255))

    
    departamentos = db.relationship('Departamento', back_populates='building', lazy=True)

    def __init__(self, cel, administrador, fecha, direccion=None, lat=None, lon=None, rfc=None,
                 num_predial=None, fecha_construccion=None, cp=None, no_deptos=None,
                 no_pisos=None, accesos_peatonales=None, accesos_vehiculares=None,
                 lugares_estacionamiento=None):
        self.cel = cel
        self.administrador = administrador
        self.fecha = fecha
        self.direccion = direccion
        self.lat = lat
        self.lon = lon
        self.rfc = rfc
        self.num_predial = num_predial
        self.fecha_construccion = fecha_construccion
        self.cp = cp
        self.no_deptos = no_deptos
        self.no_pisos = no_pisos
        self.accesos_peatonales = accesos_peatonales
        self.accesos_vehiculares = accesos_vehiculares
        self.lugares_estacionamiento = lugares_estacionamiento

    def __repr__(self):
        return f"<Building(id={self.id}, administrador='{self.administrador}', direccion='{self.direccion}')>"
