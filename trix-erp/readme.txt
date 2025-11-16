 https://dashboard.heroku.com
 http://trix-svm-backend-fd0368362b62.herokuapp.com/agenda/
 http://trix-svm-backend-fd0368362b62.herokuapp.com

 heroku container:push web -a trix-2025-app
heroku container:release web -a trix-2025-app



TRIX: Automatización Administrativa, Técnica y Legal con una Única Mensualidad
El mejor aliado para la gestión de edificios y condominios en la CDMX

¿Qué es TRIX?

TRIX es una plataforma integral que combina un ERP/CRM especializado con inteligencia de negocios y tecnología de aprendizaje automático. Diseñada para administradores y residentes, otorga transparencia total en operaciones y gestiones del día a día.
Ofrecer Opciones Flexibles

    Plan Básico y Premium: Permite a los clientes elegir entre un plan más económico con servidor compartido y un plan premium con instancias dedicadas.
    Escalabilidad: Asegura que puedas escalar los recursos fácilmente para acomodar el crecimiento de tus clientes sin comprometer la seguridad o el rendimiento.
Managed Services (Servicios Gestionados):  la instalación, configuración y posiblemente el mantenimiento continuo de aplicaciones en la infraestructura del cliente.
Software as a Service (SaaS) (Alojar el ERP en  propios servidores y ofrecer acceso remoto al cliente).


Con TRIX, es posible:

    Solicitar servicios en tiempo real
    Dar seguimiento detallado a reparaciones y mantenimiento
    Visualizar estados financieros y operativos de manera clara y sencilla

Gestión Completa de Pagos y Documentación

El portal de TRIX centraliza y facilita:

    Pagos de agua, predial, luz, CFE, entre otros
    Gestión de contratos de arrendamiento, desalojos, PDM
    Administración de datos esenciales (planos, adeudos, etc.)
    Organización de juntas vecinales con respaldo legal de la CDMX

Así, tu comunidad se mantiene en orden y preparada ante cualquier emergencia o situación que requiera documentación inmediata.

Optimización de Costos y Procesos

Al automatizar tareas administrativas, logísticas y legales, TRIX:

    Reduce costos operativos
    Mejora la eficiencia en la gestión
    Genera ahorros significativos para clientes
    Aporta insights valiosos para la toma de decisiones estratégicas


TRIX: Automatización Administrativa, Técnica y Legal con una Única Mensualidad
El mejor aliado para la gestión de edificios y condominios en la CDMX
¿Qué es TRIX?

TRIX es una plataforma integral que combina un ERP/CRM especializado con inteligencia de negocios y tecnología de aprendizaje automático. Diseñada para administradores y residentes, otorga transparencia total en operaciones y gestiones del día a día.

Con TRIX, es posible:

    Solicitar servicios en tiempo real
    Dar seguimiento detallado a reparaciones y mantenimiento
    Visualizar estados financieros y operativos de manera clara y sencilla

Asistencia Legal Integrada

Además de la optimización de procesos administrativos y logísticos, TRIX incorpora asesoría legal en derecho civil, ofreciendo:

    Apoyo en temas de convivencia vecinal, bienes y servicios
    Asesoría en trámites y asuntos legales específicos
    Cumplimiento de la normativa local en juntas vecinales y condominales

Esto garantiza que la información legal y los recursos necesarios estén disponibles para el usuario, simplificando la administración y solución de problemáticas legales.
Gestión Completa de Pagos y Documentación

El portal de TRIX centraliza y facilita:

    Pagos de agua, predial, luz, CFE, entre otros
    Gestión de contratos de arrendamiento, desalojos, PDM
    Administración de datos esenciales (planos, adeudos, etc.)
    Organización de juntas vecinales con respaldo legal de la CDMX

Así, tu comunidad se mantiene en orden y preparada ante cualquier emergencia o situación que requiera documentación inmediata.
Optimización de Costos y Procesos

Al automatizar tareas administrativas, logísticas y legales, TRIX:

    Reduce costos operativos
    Mejora la eficiencia en la gestión
    Genera ahorros significativos para clientes
    Aporta insights valiosos para la toma de decisiones estratégicas

Nuestra Propuesta de Valor

    Tecnología de punta: algoritmos de ruta óptima y respuestas automáticas para un servicio rápido y eficiente.
    Licencia de uso de software: el cliente solo necesita instalar nuestra imagen Docker para acceder a todos los módulos de TRIX.
    Consultoría y Desarrollo a la Medida: adaptamos y personalizamos el software según las necesidades específicas de cada cliente.

Con TRIX, la administración de edificios y condominios es más sencilla, transparente y segura, enfocada en un servicio de excelencia que ahorra tiempo y recursos a sus usuarios.




Bussiness inteligence
- automatiza procesos administrativos
- backend
- Dashboard
- Data insights para inteligencia de negocios y mkt
- Orientado para optimizar rutas, utlidades
- Respaldo automatizado de informacion
- PAra negocios de isntalacion y gestion de personal tecnico con ordenes de produccion
Coordinamos y unicos edificios (cuentas) con los mejores tecnicos.
AUtomatización de procesos administrativos y logitica técnica. 


Planeo que esta central se un punto para gestionar y organizar multiples edificios. las personas tendran transparencia.
las personas podran levantar tickets para orden. la orden de reparacion de manera automatica organizara la ruta 
de los tecnicos creando clusters eficientes para asignar reparaciones cerca de la unidad, mejorarndo tiempos.

En el modelo de negocio se cobrar $2,300 por depto de lo que actualmente cobran los administradores
, al ser procesos automatizados podemos disminuir esta parte. 

    - Ingreso en reparaciones
    - Ingreso en mantenimiento
    - Ingreso de venta de productos ( refacciones)



EL cliente podra ver el dashboard de su edificio, saber cuanto dinero hay en cuentas, en que se ha gastados,
cuentas por cobrar y pagar, estado de reparaciones. 

-----------------Automatizar agenda de ruta: ------------------

1.El cliente accede al front-end.
2.Solicita una reparación e ingresa su dirección.
3.Utilizando la API de Google, se obtienen las coordenadas de latitud y longitud.
4.Estos datos serán usados para la clasificación (zona norte, sur, centro).
5.Se asignará el día correspondiente, generando clústeres por zona.
6.Cada día, se atenderá un máximo de 4 ubicaciones por camioneta.
7.El algoritmo organizará las reparaciones de manera que, en un día, se atiendan todas
     las ubicaciones cercanas del norte y, en otro día, las del sur.


----------------Actual 2024: -------------------------------------------------------------------------

Es una app flask en 3 contenedores docker TRIX_Backend_container, TRIX_db_container y TRIX_phpmyadmin_container y TRIX_redis_container.
Esta pensada para ser el centro de control TRIX. Empresa de automatizacion de puertas, mi principal cliente son casa resideciales
, el modelo de negocio es instalar, mantenimiento , productos.

Funciones principales 

1.Generación de rutas diarias: Utiliza las coordenadas (latitud y longitud) de las direcciones programadas 
    y una función de vecino cercano para optimizar las rutas de servicio.

2.Gestión de clientes: Almacena información como teléfono, dirección, nombre, coordenadas y
     clasificación de zona, esta última obtenida mediante un modelo de Máquina de Soporte Vectorial (SVM).


3.Analis temporal de dinero  (ventas, pagos, utilidad) genera predicciones con diferentes métodos incluyendo 
    machinelearning. 

Servicios legales
civil
vecinos




Módulos de la aplicación:
        -semana actual , se registra el estado de las visitas de esa semana.
        -Historico ventas
        -Pago semanal 
        -Historico pagos
        -Mapa folium
        -Mapa mapbox
        -Cotizacion automática
        -Agenda de clientes ( base de datos clientes)
        -Dashboard ( analisis de dinero en el tiempo)

automatizaciontrix-backend/
├── app/
│   ├── __init__.py
│   ├── routes
│   │   ├── __init__.py
│   │   └── agenda.py
│   │   └── login.py
│   │   └── main.py
│   │   └── cotizar.py
│   ├── models/
│   │   ├── __init__.py
│   │   └── user.py
│   │   └── svm_model.pkl
│   ├── utils/
│   │   └── prediction.py
│   │   └── correo.py
│   ├── static/
│   │   ├── css
│   │   ├── img 
│   │   |    └ productos
│   │   └── js
│   │        └agenda.js
│   │   
│   ├── templates/
│   │       └── main.html
│   └── scripts/
│       └── cotizacion.py
│       └── decorators.py
├── tests/
│   └── test_routes.py
├── config/
│   └── config.py
├── migrations/
├── .gitignore
├── Procfile
├── .env
├── Dockerfile 
├── entrypoint.sh
├── requirements.txt
├── README.md
└── run.py

Este backend flask para recibir solicitudes para clasificar lat y lon. 
    automatizar este proceso
    Heroku, Github, Dockerhub

1. Repositorio en Github
    git add .
    git commit -m "nombre"
    git push origin main

2. contruir imagen y 

    docker build -t joselhuidobro/trix-svm:v12024-2025 .
    docker tag joselhuidobro/trix-svm:v12024-2025 registry.heroku.com/trix-svm-backend/web
    heroku container:login
    docker push registry.heroku.com/trix-svm-backend/web

4.Liberar aplicación

    heroku container:release web -a trix-svm-backend
    heroku config:set FLASK_APP=run.py FLASK_ENV=production --app trix-svm-backend
    heroku logs --tail --app trix-svm-backend
    


para accesar a base de datos heroku postgressql
    postgresql-reticulated-72468

comando para crear base postgresql gratis en heroku
        heroku addons:create heroku-postgresql:hobby-dev --app trix-svm-backend

Usar la Interfaz de Línea de Comandos (CLI) de Heroku con psql

heroku pg:psql -a trix-svm-backend

\d agenda
\d cotizaciones_historico
password = 'Yeikam0sha_57492874'


trix-svm-backend::DATABASE=> \d agenda
                                         Tabla ½public.agenda╗
    Columna    |          Tipo          | Ordenamiento | Nulable  |            Por omisi¾n
---------------+------------------------+--------------+----------+------------------------------------
 id            | integer                |              | not null | nextval('agenda_id_seq'::regclass)
 dia           | character varying(20)  |              |          |
 direccion     | text                   |              |          |
 clasificacion | character varying(50)  |              |          |
 lat           | double precision       |              |          |
 lon           | double precision       |              |          |
 fecha         | date                   |              |          |
 celular       | character varying(15)  |              |          |
 horario       | time without time zone |              |          |
═ndices:
    "agenda_pkey" PRIMARY KEY, btree (id)

leer agenda
SELECT * FROM agenda;
SELECT * FROM usuarios;
SELECT * FROM vecinos;

SELECT * FROM ordenes_semana;
SELECT * FROM cotizaciones_historico;


borrar contenido de agenda
DELETE FROM agenda;


1.-- Crear tabla de usuarios
CREATE TABLE usuarios (
    id INT AUTO_INCREMENT PRIMARY KEY,
    nombre VARCHAR(100) NOT NULL,
    telefono VARCHAR(15),
    nivel_seguridad TINYINT NOT NULL,
    correo VARCHAR(100) UNIQUE NOT NULL
);




2.-- Crear tabla de movimientos en base de datos
CREATE TABLE movimientos (
    id INT AUTO_INCREMENT PRIMARY KEY,
    usuario_id INT NOT NULL,
    nombre VARCHAR(100) NOT NULL,
    telefono VARCHAR(15),
    nivel_seguridad TINYINT NOT NULL,
    correo VARCHAR(100),
    fecha_movimiento DATETIME NOT NULL DEFAULT CURRENT_TIMESTAMP,
    tipo_movimiento ENUM('entrada', 'salida') NOT NULL,
    FOREIGN KEY (usuario_id) REFERENCES usuarios(id) ON DELETE CASCADE
);

-- Crear tabla de almacén de materiales
CREATE TABLE almacen_materiales (id SERIAL PRIMARY KEY, nombre_material VARCHAR(100) NOT NULL, cantidad INT NOT NULL, unidad_medida VARCHAR(50) NOT NULL, fecha_ingreso TIMESTAMP DEFAULT CURRENT_TIMESTAMP);


-- Crear tabla de almacén de herramientas
CREATE TABLE almacen_herramientas (id SERIAL PRIMARY KEY, nombre_herramienta VARCHAR(100) NOT NULL, cantidad INT NOT NULL, estado VARCHAR(20) DEFAULT 'nueva' CHECK (estado IN ('nueva', 'usada', 'reparación')), fecha_ingreso TIMESTAMP DEFAULT CURRENT_TIMESTAMP);

\dt  ( listar tablas)

                  Listado de relaciones
 Esquema |        Nombre        | Tipo  |     Due±o
---------+----------------------+-------+----------------
 public  | agenda               | tabla | u7vqat0fodim1i
 public  | almacen_herramientas | tabla | u7vqat0fodim1i
 public  | almacen_materiales   | tabla | u7vqat0fodim1i
 public  | movimientos          | tabla | u7vqat0fodim1i
 public  | usuarios             | tabla | u7vqat0fodim1i


# col, resultados: id,dia,direccion,clasificacion,lat,lon,fecha,celular,horario


crear dato en usuarios

Rafael Ramos martinez 55-54-108-587 rafael75r@gmail.com administrador


datos falsos

DATABASE_URL=postgres://u7vqat3333333333333333333333g7.us-east-1.rds.amazonaws.com:5432/da1111111b8t
EMAIL_USER=joselhuidobro@gmail.com
EMAIL_PASSWORD=cz11111skl34223
DB_USER=u7v11111111
DB_PASSWORD=p3b293434343434
DB_HOST=cbdhrt3434343434s.amazonaws.com
DB_PORT=5432
DB_NAME=daaqaaec34343434
SECRET_KEY=tu_clave_secreta_muy_segura_y_unica




                               Tabla ½public.ordenes_semana╗
     Columna     |          Tipo          | Ordenamiento | Nulable  |                Por omisi¾n
-----------------+------------------------+--------------+----------+--------------------------------------------
 id              | integer                |              | not null | nextval('ordenes_semana_id_seq'::regclass)
 cel             | character varying(20)  |              | not null |
 nombre          | character varying(100) |              | not null |
 fecha           | date                   |              | not null |
 direccion       | character varying(255) |              |          |
 concepto        | character varying(100) |              |          |
 categoria       | character varying(50)  |              |          |
 no_semana       | integer                |              |          |
 dia_semana      | character varying(20)  |              |          |
 cantidad        | integer                |              |          | 0
 precio_unitario | numeric(10,2)          |              |          | 0.00
 total_cotizado  | numeric(12,2)          |              |          | 0.00
 total_pagado    | numeric(12,2)          |              |          | 0.00
 forma_de_pago   | character varying(50)  |              |          |
 estado          | character varying(50)  |              |          | 'Pendiente'::character varying
 h_llegada       | time without time zone |              |          |
 accion          | character varying(100) |              |          |
 cambiar_estado  | boolean                |              |          | false
 horario         | character varying(50)  |              |          |
 unidad_trix     | character varying(50)  |              |          |
 lat             | numeric(9,6)           |              |          |
 lon             | numeric(9,6)           |              |          |


kubectl apply -f deployment.yaml -f service.yaml




