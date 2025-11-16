from flask import Flask
from flask_cors import CORS
from config.config import Config
from flask_login import LoginManager, UserMixin
from dotenv import load_dotenv
from db2 import db, get_database_uri
import os
from .models import User
from flask_wtf import CSRFProtect
import logging
from flask_compress import Compress
from werkzeug.middleware.proxy_fix import ProxyFix   # ← nuevo

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger(__name__)
logger.info("Bienvenido")

login_manager = LoginManager()
csrf = CSRFProtect()  # ← muévelo arriba para usar una sola instancia

@login_manager.user_loader
def load_user(user_id):
    return User.query.get(int(user_id))

def create_app():
    app = Flask(__name__)
    app.wsgi_app = ProxyFix(app.wsgi_app, x_for=1, x_proto=1, x_host=1, x_prefix=1)

    # 1) Cargar .env ANTES de leer config/envs
    load_dotenv()

    CORS(app)

    # 2) Claves y cookies (por defecto, seguras en HTTP)
    app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY', 'dev-change-me')
    app.config['WTF_CSRF_SECRET_KEY'] = os.environ.get('WTF_CSRF_SECRET_KEY', app.config['SECRET_KEY'])
    # 0 => sin expiración; cualquier otro => segundos (por defecto 3600)
    app.config['WTF_CSRF_TIME_LIMIT'] = None if os.environ.get('WTF_CSRF_TIME_LIMIT') == '0' else 3600

    # Para HTTP directo: Lax + Secure=False
    app.config['SESSION_COOKIE_SAMESITE'] = os.environ.get('SESSION_COOKIE_SAMESITE', 'Lax')
    app.config['SESSION_COOKIE_SECURE'] = os.environ.get('SESSION_COOKIE_SECURE', '0') in ('1','true','True')
    app.config['SESSION_COOKIE_PATH'] = os.environ.get('SESSION_COOKIE_PATH', '/')

    # 3) Resto de config
    app.config.from_object(Config)
    app.config['TEMPLATES_AUTO_RELOAD'] = True
    app.jinja_env.auto_reload = True

    # 4) DB
    app.config['SQLALCHEMY_DATABASE_URI'] = get_database_uri()
    app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

    # 5) Extensiones
    login_manager.init_app(app)
    login_manager.login_view = 'login.login'
    login_manager.login_message = "Por favor, inicia sesión para acceder a esta página."
    login_manager.login_message_category = "warning"

    csrf.init_app(app)
    db.init_app(app)

    # 6) Blueprints
    from app.routes import main, agenda, login_bp, mapa, cotizar, building, vecinos, departamento_bp, map_box, pago, dashboard_bp, inmobiliaria_bp
    app.register_blueprint(main, url_prefix='/')
    app.register_blueprint(agenda, url_prefix='/agenda')
    app.register_blueprint(mapa, url_prefix='/mapa')
    app.register_blueprint(login_bp, url_prefix='/login')
    app.register_blueprint(cotizar, url_prefix='/cotizar')
    app.register_blueprint(building, url_prefix='/building')
    app.register_blueprint(vecinos, url_prefix='/vecinos')
    app.register_blueprint(departamento_bp, url_prefix='/departamento')
    app.register_blueprint(map_box, url_prefix='/map3')
    app.register_blueprint(pago, url_prefix='/pago')
    app.register_blueprint(dashboard_bp, url_prefix='/dashboard')
    app.register_blueprint(inmobiliaria_bp, url_prefix='/inmobiliaria')

    return app

