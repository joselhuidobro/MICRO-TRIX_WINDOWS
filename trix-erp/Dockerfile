# Dockerfile

# Usar una imagen base oficial de Python
FROM python:3.9-slim

# Establecer el directorio de trabajo en /app
WORKDIR /app

# Instalar dependencias del sistema necesarias
RUN apt-get update && apt-get install -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copiar el archivo requirements.txt primero para aprovechar la caché de Docker
COPY requirements.txt .

# Actualizar pip y luego instalar las dependencias de Python
RUN pip install --upgrade pip
RUN pip install --no-cache-dir -r requirements.txt

# Copiar el resto de la aplicación
COPY . .

#EXPOSE 8000

# Definir variables de entorno
ENV FLASK_APP=run.py
ENV FLASK_DEBUG=0  



# Copiar el entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Definir el entrypoint
ENTRYPOINT ["/entrypoint.sh"]