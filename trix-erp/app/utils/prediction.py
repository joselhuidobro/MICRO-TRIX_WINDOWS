# app/utils/prediction.py

import joblib
import os
import numpy as np
from app.utils.constants import CLASS_MAPPING  # Solo si usas constants.py

# Ruta al modelo SVM
MODEL_PATH = os.path.join(os.path.dirname(__file__), '..', 'models', 'svm_model.pkl')

# Cargar el modelo una sola vez
model = joblib.load(MODEL_PATH)

def predict_location(lat, lon):
    """
    Realiza una predicción basada en la latitud y longitud proporcionadas.

    Args:
        lat (float): Latitud.
        lon (float): Longitud.

    Returns:
        str: Clasificación de la zona ('TRIX_Norte', 'TRIX_Centro-sur', 'TRIX_Sur-este').
    """
    # Convertir las entradas en un array numpy
    features = np.array([[lat, lon]])

    # Realizar la predicción
    prediction = model.predict(features)

    # Convertir a un entero de Python
    clasificacion_num = int(prediction[0])

    # Obtener la descripción de la clasificación
    clasificacion = CLASS_MAPPING.get(clasificacion_num, 'Clasificación desconocida')

    return clasificacion
