from flask import Blueprint, render_template, request, redirect, url_for, flash
from flask_login import login_user, logout_user, current_user, login_required

from app.models import User 
from db2 import db
from werkzeug.security import generate_password_hash
import re  # Para validar el formato del correo electrónico

login_bp = Blueprint('login', __name__)

@login_bp.route('/', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        email = request.form.get('email')
        password = request.form.get('password')

        # Buscar al usuario por correo electrónico
        user = User.query.filter_by(correo=email).first()
        if user:
            print(f"Usuario encontrado: {user.nombre}")
            # Verificar la contraseña
            if user.check_password(password):
                login_user(user)
                flash('Inicio de sesión exitoso.', 'success')
                if user.nivel_seguridad == 0:
                    return redirect(url_for('mapbox.mapa_box'))
                # Redirigir según el nivel de seguridad a visitants lo mandamos a buildings
                elif  user.nivel_seguridad == 1:
                    return redirect(url_for('building.building_route'))
                else:
                    return redirect(url_for('agenda.agenda_route'))
            else:
                flash('Contraseña incorrecta.', 'danger')
        else:
            flash('El correo no está registrado.', 'danger')
    
    # Si la solicitud es GET o hay errores en el POST, renderizar la plantilla de login
    return render_template('login.html')

@login_bp.route('/logout')
def logout():
    logout_user()
    flash('Sesión cerrada.', 'info')
    return redirect(url_for('login.login'))  # Redirige a la página de login


@login_bp.route('/register', methods=['GET', 'POST'])
def register():
    if request.method == 'POST':
        return guardar_usuario(request.form)
    return render_template('register.html')

def guardar_usuario(form_data):
    # Obtener los datos del formulario
    rfc = form_data.get('rfc')
    razon_social = form_data.get('razon_social')
    cel = form_data.get('cel')
    correo = form_data.get('correo')
    password = form_data.get('password')

    # Validar los datos 
    if not correo or not password or not rfc or not razon_social:
        flash('Todos los campos son obligatorios.')
        return redirect(url_for('login.register'))

    if not re.match(r"[^@]+@[^@]+\.[^@]+", correo):
        flash('El correo electrónico no es válido.')
        return redirect(url_for('login.register'))

    if len(password) < 8:
        flash('La contraseña debe tener al menos 8 caracteres.')
        return redirect(url_for('login.register'))

    # Verificar si el correo electrónico ya existe
    existing_user = User.query.filter_by(correo=correo).first()
    if existing_user:
        flash('Ya existe una cuenta con este correo electrónico.')
        return redirect(url_for('login.register'))

    nivel_seguridad = 3  # O la lógica que necesites para asignar el nivel

    try:
        # Crear una nueva instancia del modelo User
        nuevo_usuario = User(
            rfc=rfc,
            razon_social=razon_social,
            cel=cel,
            correo=correo,
            nivel_seguridad=nivel_seguridad
        )

        # Establecer la contraseña
        nuevo_usuario.set_password(password)

        # Guardar en la base de datos
        db.session.add(nuevo_usuario)
        db.session.commit()

        # Iniciar sesión al usuario (opcional)
        login_user(nuevo_usuario)

        flash('¡Registro exitoso! Bienvenido.')
        return redirect(url_for('agenda.agenda_route'))  # Redirigir a la página principal
    except Exception as e:
        # Manejar la excepción (registrar el error, mostrar un mensaje al usuario, etc.)
        print(f"Error al registrar usuario: {e}")
        flash('Ocurrió un error al registrar el usuario.')
        return redirect(url_for('login.register'))
    

@login_bp.route('/guest', methods=['GET'])
def login_guest():
    user = User.query.filter_by(correo='invitado@trix.com').first()
    if not user:
        flash('Cuenta de invitado no disponible.', 'danger')
        return redirect(url_for('login.login'))

    # No cambiar nivel aquí. En la BD debe ser 0.
    login_user(user, remember=False, fresh=True)
    flash('Sesión de invitado activa.', 'info')
    return redirect(url_for('mapbox.mapa_box'))
