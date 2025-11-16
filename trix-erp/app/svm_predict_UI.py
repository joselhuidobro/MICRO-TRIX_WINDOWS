import pickle
import numpy as np

def load_model(file_path):
    with open(file_path, 'rb') as f:
        return pickle.load(f)



def predict_lat_lon(lat, lon):
    # Cargamos el modelo
    svm_model = load_model('/usr/src/app/scripts/ia/svm/svm_model.pkl')
    
    class_mapping = {1: 'TRIX_Norte', 2: 'TRIX_Centro-sur', 3: 'TRIX_Sur-este'}
    # Hacemos la predicción
    lat = np.array([lat]).reshape(-1, 1)
    lon = np.array([lon]).reshape(-1, 1)
    prediction = svm_model.predict(np.hstack((lat, lon)))
    print(prediction)
        
    # Traducimos la predicción a su nombre de clase correspondiente
    predicted_class = class_mapping.get(prediction[0], 'Clase desconocida')
        
    return f'La clasificacion de zona es: {predicted_class}'