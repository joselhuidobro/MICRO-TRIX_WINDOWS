from db2 import db
from flask_login import UserMixin
from werkzeug.security import generate_password_hash, check_password_hash

class User(UserMixin, db.Model):
    __tablename__ = 'usuarios'
    id = db.Column(db.Integer, primary_key=True)
    nombre = db.Column(db.String(150))
    telefono = db.Column(db.String(50))
    nivel_seguridad = db.Column(db.Integer)
    correo = db.Column(db.String(150), unique=True, nullable=False)
    password_hash = db.Column(db.Text)  # Cambiar a db.Text para evitar limitaciones
    rfc = db.Column(db.String(255))  # Agregar columna RFC
    razon_social = db.Column(db.String(255))  # Agregar columna RAZON_SOCIAL
    cel = db.Column(db.String(255))  # Agregar columna cel

    # Métodos para manejar la contraseña
    def set_password(self, password):
        self.password_hash = generate_password_hash(password)

    def check_password(self, password):
        return check_password_hash(self.password_hash, password)



