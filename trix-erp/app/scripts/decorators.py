from functools import wraps
from flask import redirect, url_for, flash
from flask_login import current_user

def nivel_requerido(*niveles):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            if current_user.is_authenticated and current_user.nivel_seguridad in niveles:
                return func(*args, **kwargs)
            else:
                flash("No tienes el nivel necesario para acceder a esta página.", "warning")
                return redirect(url_for('agenda.agenda_route'))
        return wrapper
    return decorator
