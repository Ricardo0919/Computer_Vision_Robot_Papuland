# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Reentrenamiento del modelo de detección de semáforo 
#                             con dataset propio y data augmentation
# Materia: Implementación de Robótica Inteligente
# Fecha: 12 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

from ultralytics import YOLO  # Librería principal para usar modelos YOLOv8
import torch                  # PyTorch: backend que usa YOLOv8 para entrenamiento
import multiprocessing        # Necesario en Windows, no afecta en Linux

def main():
    # Verifica si CUDA (GPU) está disponible para acelerar el entrenamiento
    print("CUDA disponible:", torch.cuda.is_available())

    # Carga del modelo previamente entrenado (best.pt) para reentrenamiento
    model = YOLO('models/best.pt')  # Ruta al modelo base

    # Reentrenamiento del modelo con data augmentation personalizado
    model.train(
        data='data.yaml',                    # Configuración del dataset
        epochs=60,                           # Número de épocas
        imgsz=160,                           # Tamaño de las imágenes
        batch=32,                            # Tamaño del lote
        device=0,                            # GPU
        name='traffic_light_retrained',      # Nombre de la corrida
        augment=True,                        # Activar data augmentation
        degrees=10,                          # Rotación aleatoria ±10°
        scale=0.5,                           # Escalado aleatorio (0.5 a 1.5)
        shear=10,                            # Shear horizontal/vertical ±10°
        perspective=0.0001,                  # Distorsión de perspectiva leve
        flipud=0.0,                          # No voltear verticalmente
        fliplr=0.5,                          # 10% de voltear horizontal
        hsv_h=0.015,                         # Variación de tono
        hsv_s=0.7,                           # Variación de saturación
        hsv_v=0.4                            # Variación de brillo
    )

# Punto de entrada del script
if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
