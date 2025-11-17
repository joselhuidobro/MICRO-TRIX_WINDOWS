# correo.py

import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from dotenv import load_dotenv, find_dotenv
import os
from datetime import datetime, timedelta
import uuid
import pytz

def correo(df_nuevo):
    # Cargar las variables de entorno
    load_dotenv(find_dotenv())

    email_user = os.getenv("EMAIL_USER")
    email_password = os.getenv("EMAIL_PASSWORD")
    email_destinatario = "joselhuidobro@gmail.com"

    # Verificar que las variables de entorno se cargaron correctamente
    if not email_user or not email_password:
        print("Error: EMAIL_USER o EMAIL_PASSWORD no están definidos en el archivo .env")
        return

    # Extraer datos del nuevo cliente
    direccion = df_nuevo.iloc[0]['direccion']
    clasificacion = df_nuevo.iloc[0]['clasificacion']
    lat = df_nuevo.iloc[0]['lat']
    lon = df_nuevo.iloc[0]['lon']
    fecha = df_nuevo.iloc[0]['fecha']
    horario = df_nuevo.iloc[0]['horario']

    # Obtener la zona horaria de CDMX
    cdmx_tz = pytz.timezone('America/Mexico_City')

    if fecha and horario:
        # Parsear 'fecha' y 'horario' para obtener un objeto datetime
        try:
            start_time = datetime.strptime(f"{fecha} {horario}", "%Y-%m-%d %H:%M")
            start_time = cdmx_tz.localize(start_time)
        except ValueError:
            start_time = datetime.now(cdmx_tz) + timedelta(hours=1)
    else:
        start_time = datetime.now(cdmx_tz) + timedelta(hours=1)

    end_time = start_time + timedelta(hours=1)  # Duración de 1 hora

    # Crear el mensaje de correo electrónico como multipart
    mensaje = MIMEMultipart('mixed')
    mensaje['Subject'] = 'Nuevo cliente añadido a la agenda'
    mensaje['From'] = email_user
    mensaje['To'] = email_destinatario

    # Contenido del correo
    contenido = f"""
    Estimado/a,

    Se ha añadido un nuevo cliente a su agenda.

    Detalles del cliente:
    - Dirección: {direccion}
    - Clasificación: {clasificacion}
    - Latitud: {lat}
    - Longitud: {lon}
    - Fecha y hora: {start_time.strftime('%d de %B de %Y a las %I:%M %p')}

    Por favor, confirme la recepción de este correo.

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
SUMMARY:Visita a nuevo cliente
LOCATION:{direccion}
DESCRIPTION:Visita programada con el nuevo cliente
END:VEVENT
END:VCALENDAR
"""

    # Crear el adjunto .ics como MIMEText
    ics_attachment = MIMEText(ics_content, 'calendar')

    # Establecer las cabeceras del adjunto
    ics_attachment.add_header('Content-Disposition', 'attachment', filename='evento.ics')

    # Adjuntar el archivo .ics al mensaje
    mensaje.attach(ics_attachment)

    # Enviar el correo electrónico
    try:
        with smtplib.SMTP_SSL('smtp.gmail.com', 465) as smtp:
            smtp.login(email_user, email_password)
            smtp.send_message(mensaje)
        print('Correo enviado exitosamente.')
    except Exception as e:
        print(f'Error al enviar el correo: {e}')