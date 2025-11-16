# ordenes_semana.py

from db2 import db
from datetime import datetime

class Historico_ventas(db.Model):
    __tablename__ = 'historico_ventas'
    id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    cel = db.Column(db.String(20), nullable=False)
    nombre = db.Column(db.String(100), nullable=False)
    fecha = db.Column(db.Date, nullable=False, default=datetime.utcnow)
    direccion = db.Column(db.String(255))
    concepto = db.Column(db.String(100))
    categoria = db.Column(db.String(50))
    no_semana = db.Column(db.Integer)
    dia_semana = db.Column(db.String(20))
    cantidad = db.Column(db.Integer, default=0)
    precio_unitario = db.Column(db.Numeric(10, 2), default=0.00)
    total_cotizado = db.Column(db.Numeric(12, 2), default=0.00)
    total_pagado = db.Column(db.Numeric(12, 2), default=0.00)
    forma_de_pago = db.Column(db.String(50))
    estado = db.Column(db.String(50), default='pendiente')
    h_llegada = db.Column(db.Time)
    accion = db.Column(db.String(100))
    cambiar_estado = db.Column(db.Boolean, default=False)
    horario = db.Column(db.String(50))
    unidad_trix = db.Column(db.String(50))
    lat = db.Column(db.Float)
    lon = db.Column(db.Float)

    def __init__(self, cel, nombre, fecha, direccion=None, concepto=None, categoria=None,
                 no_semana=None, dia_semana=None, cantidad=0, precio_unitario=0.00,
                 total_cotizado=0.00, total_pagado=0.00, forma_de_pago=None,
                 estado='pendiente', h_llegada=None, accion=None,
                 cambiar_estado=False, horario=None, unidad_trix=None, lat=None, lon=None):
        self.cel = cel
        self.nombre = nombre
        self.fecha = fecha
        self.direccion = direccion
        self.concepto = concepto
        self.categoria = categoria
        self.no_semana = no_semana
        self.dia_semana = dia_semana
        self.cantidad = cantidad
        self.precio_unitario = precio_unitario
        self.total_cotizado = total_cotizado
        self.total_pagado = total_pagado
        self.forma_de_pago = forma_de_pago
        self.estado = estado
        self.h_llegada = h_llegada
        self.accion = accion
        self.cambiar_estado = cambiar_estado
        self.horario = horario
        self.unidad_trix = unidad_trix
        self.lat = lat
        self.lon = lon

    def __repr__(self):
        return f"<historico_ventas(id={self.id}, nombre='{self.nombre}', fecha={self.fecha})>"
