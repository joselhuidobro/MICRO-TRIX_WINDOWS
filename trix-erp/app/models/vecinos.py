from db2 import db
from datetime import datetime

class Vecinos(db.Model):
    __tablename__ = 'vecinos'

    id = db.Column(db.Integer, primary_key=True)  # Define 'id' as primary key
    id_edificio = db.Column(db.Integer, db.ForeignKey('buildings.id'), nullable=False)  # Foreign key to 'edificios' 
    id_depto = db.Column(db.Integer, db.ForeignKey('departamentos.id'), nullable=False)  # Foreign key to 'departamentos' 
    
    direccion = db.Column(db.String(255))
    lat = db.Column(db.Float)
    lon = db.Column(db.Float)
    interior = db.Column(db.String(10))
    piso = db.Column(db.Integer)
    cel = db.Column(db.String(20))
    rfc = db.Column(db.String(20))
    correo = db.Column(db.String(255))
    nombre = db.Column(db.String(255), nullable=False)
    password = db.Column(db.String(255), nullable=False)
    nivel_seguridad = db.Column(db.Integer)
    razon_social = db.Column(db.String(255))


    id_depto = db.Column(db.Integer, db.ForeignKey('departamentos.id'), nullable=False)
    departamento = db.relationship('Departamento', back_populates='vecinos') 

    def __init__(self, id_edificio, id_depto, nombre, password, direccion=None, lat=None, lon=None, 
                 interior=None, piso=None, cel=None, rfc=None, correo=None, nivel_seguridad=None, razon_social=None):
        self.id_edificio = id_edificio
        self.id_depto = id_depto
        self.direccion = direccion
        self.lat = lat
        self.lon = lon
        self.interior = interior
        self.piso = piso
        self.cel = cel
        self.rfc = rfc
        self.correo = correo
        self.nombre = nombre
        self.password = password
        self.nivel_seguridad = nivel_seguridad
        self.razon_social = razon_social

    def __repr__(self):
        return f"<Vecino {self.nombre} - Depto {self.id_depto} - Edificio {self.id_edificio}>"