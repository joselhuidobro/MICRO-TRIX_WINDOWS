# syntax=docker/dockerfile:1

FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# Dependencias de runtime que requieren los wheels de OpenCV
# (libGL para algunos backends, ffmpeg para video I/O)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglib2.0-0 libsm6 libxext6 libxrender1 libgl1 \
    ffmpeg \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Instala NumPy + OpenCV (HEADLESS por defecto)
# Cambia a "opencv-python" si necesitas GUI (imshow, HighGUI)
ARG OPENCV_FLAVOR=opencv-python-headless
# Si quieres fijar versión: --build-arg OPENCV_VERSION=4.10.0.84
ARG OPENCV_VERSION=
RUN pip install --no-cache-dir --upgrade pip \
 && if [ -z "$OPENCV_VERSION" ]; then \
        pip install --no-cache-dir numpy ${OPENCV_FLAVOR}; \
    else \
        pip install --no-cache-dir numpy ${OPENCV_FLAVOR}==${OPENCV_VERSION}; \
    fi

# Copia tu app si la tienes (opcional)
# COPY . .

# Comando por defecto: imprime versión y sale
CMD ["python", "-c", "import cv2; import numpy as np; print('OpenCV:', cv2.__version__)"]

