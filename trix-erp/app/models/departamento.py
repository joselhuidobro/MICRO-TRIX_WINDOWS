from db2 import db
from datetime import datetime

class Departamento(db.Model):
    __tablename__ = 'departamentos'

    id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    no_del_depto = db.Column(db.String(10), nullable=False)  # Número del departamento (e.g., "A101")
    piso = db.Column(db.Integer, nullable=False)  # Piso donde se encuentra el departamento

    cel = db.Column(db.String(20), nullable=False)  # Número de celular del propietario o responsable
    contacto = db.Column(db.String(100), nullable=False)
    rfc = db.Column(db.String(20), nullable=True)  # RFC del dueño (opcional)
    correo = db.Column(db.String(255), nullable=True)  # Correo electrónico del propietario o responsable
    
    no_estacionamientos = db.Column(db.Integer, nullable=False, default=0)  # Número de estacionamientos asignados
    roof_garden = db.Column(db.Boolean, nullable=False, default=False)  # Indica si tiene roof garden
    puerta_automatica = db.Column(db.Boolean, nullable=False, default=False)  # Indica si tiene puerta automática

    no_cuartos = db.Column(db.Integer, nullable=False, default=1)  # Número de cuartos en el departamento

    # Relación con el modelo Building
    building_id = db.Column(db.Integer, db.ForeignKey('buildings.id'), nullable=False)

    building = db.relationship('Building', back_populates='departamentos', lazy=True)

    vecinos = db.relationship('Vecinos', back_populates='departamento')

    def __init__(self, no_del_depto, piso, cel, contacto, building_id, rfc=None, correo=None,
                 no_estacionamientos=0, roof_garden=False, puerta_automatica=False,
                 no_cuartos=1):
        
        self.no_del_depto = no_del_depto
        self.piso = piso
        self.cel = cel
        self.contacto = contacto
        self.building_id = building_id
        self.rfc = rfc
        self.correo = correo
        self.no_estacionamientos = no_estacionamientos
        self.roof_garden = roof_garden
        self.puerta_automatica = puerta_automatica
        self.no_cuartos = no_cuartos

    def __repr__(self):
        return f"<Departamento {self.no_del_depto} - Piso {self.piso} - Building ID {self.building_id}>"
