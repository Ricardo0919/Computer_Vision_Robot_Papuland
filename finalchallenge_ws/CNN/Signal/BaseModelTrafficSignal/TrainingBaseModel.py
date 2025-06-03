# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Entrenamiento de un modelo base para detección de señales de transito
# Materia: Implementación de Robótica Inteligente
# Fecha: 12 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

from ultralytics import YOLO  # Librería principal para usar modelos YOLOv8
import torch                  # PyTorch: backend que usa YOLOv8 para entrenar
import multiprocessing        # Necesario en Windows, no afecta en Linux

def main():
    # Verifica si CUDA (GPU) está disponible para acelerar el entrenamiento
    print("CUDA disponible:", torch.cuda.is_available())

    # Carga del modelo YOLOv8 preentrenado ("s" = small)
    # Este modelo servirá como base y se ajustará al dataset específico de semáforos
    model = YOLO('yolov8s.pt')

    # Inicio del proceso de entrenamiento
    model.train(
        data='data.yaml',                    # Configuración del dataset
        epochs=60,                           # Número de épocas
        imgsz=160,                           # Tamaño de las imágenes
        batch=32,                            # Tamaño del lote
        device=0,                            # GPU
        name='traffic_signals_base_model',   # Nombre de la carpeta de resultados
    )

# Punto de entrada del script
# freeze_support se requiere para compatibilidad en Windows; en Linux no afecta
if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
