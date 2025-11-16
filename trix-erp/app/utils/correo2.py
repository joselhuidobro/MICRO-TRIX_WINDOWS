# correo2.py

import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.application import MIMEApplication
from dotenv import load_dotenv, find_dotenv
import os
from datetime import datetime, timedelta
import uuid
import pytz

def correo2(direccion, lat, lon, nombre, fecha, concepto_list, clasificacion, archivo=None):
    # Cargar las variables de entorno
    load_dotenv(find_dotenv())

    email_user = os.getenv("EMAIL_USER")
    email_password = os.getenv("EMAIL_PASSWORD")
    email_destinatario = "automatizaciontrix@gmail.com"

    # Verificar que las variables de entorno se cargaron correctamente
    if not email_user or not email_password:
        print("Error: EMAIL_USER o EMAIL_PASSWORD no están definidos en el archivo .env")
        return

    # Procesar 'fecha' si es necesario
    # Asumiendo que 'fecha' es una cadena en formato 'YYYY-MM-DD'
    try:
        start_time = datetime.strptime(fecha, '%Y-%m-%d')
        cdmx_tz = pytz.timezone('America/Mexico_City')
        start_time = cdmx_tz.localize(start_time)
    except ValueError:
        cdmx_tz = pytz.timezone('America/Mexico_City')  # Definir antes de usar en except
        start_time = datetime.now(cdmx_tz) + timedelta(hours=1)

    end_time = start_time + timedelta(hours=1)  # Duración de 1 hora

    # Crear el mensaje de correo electrónico como multipart
    mensaje = MIMEMultipart('mixed')
    mensaje['Subject'] = 'Nueva Cotización Generada'
    mensaje['From'] = email_user
    mensaje['To'] = email_destinatario

    # Contenido del correo
    concepto_detallado = '\n'.join([f"- {producto}" for producto in concepto_list])

    contenido = f"""
    Estimado/a,

    Se ha generado una nueva cotización.

    Detalles de la cotización:
    - Nombre: {nombre}
    - Dirección: {direccion}
    - Latitud: {lat}
    - Longitud: {lon}
    - Fecha y hora: {start_time.strftime('%d de %B de %Y a las %I:%M %p')}
    - Clasificación: {clasificacion}
    - Concepto:
    {concepto_detallado}

    Por favor, revise los detalles de la cotización.

    Atentamente,
    Su Equipo
    """

    # Adjuntar el contenido de texto plano al mensaje
    mensaje.attach(MIMEText(contenido, 'plain'))

    # Generar el archivo .ics
    uid = str(uuid.uuid4())
    dtstamp = datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
    dtstart = start_time.strftime('%Y%m%dT%H%M%SZ')
    dtend = end_time.strftime('%Y%m%dT%H%M%SZ')

    ics_content = f"""BEGIN:VCALENDAR
VERSION:2.0
PRODID:-//Su Empresa//ES
METHOD:REQUEST
BEGIN:VEVENT
UID:{uid}
DTSTAMP:{dtstamp}
DTSTART:{dtstart}
DTEND:{dtend}
SUMMARY:Cotización Generada
LOCATION:{direccion}
DESCRIPTION:Cotización creada por {nombre}
END:VEVENT
END:VCALENDAR
"""

    # Crear el adjunto .ics como MIMEText
    ics_attachment = MIMEText(ics_content, 'calendar')
    ics_attachment.add_header('Content-Disposition', 'attachment', filename='evento.ics')
    mensaje.attach(ics_attachment)

    # Adjuntar el archivo .docx si existe
    if archivo and os.path.exists(archivo):
        try:
            with open(archivo, 'rb') as f:
                part = MIMEApplication(f.read(), Name=os.path.basename(archivo))
            part['Content-Disposition'] = f'attachment; filename="{os.path.basename(archivo)}"'
            mensaje.attach(part)
        except Exception as e:
            print(f"Error al adjuntar el archivo: {e}")

    # Enviar el correo electrónico
    try:
        with smtplib.SMTP_SSL('smtp.gmail.com', 465) as smtp:
            smtp.login(email_user, email_password)
            smtp.send_message(mensaje)
        print('Correo enviado exitosamente.')
    except Exception as e:
        print(f'Error al enviar el correo: {e}')
