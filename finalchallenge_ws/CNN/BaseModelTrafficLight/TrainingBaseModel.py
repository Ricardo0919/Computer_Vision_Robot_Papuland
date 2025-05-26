# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Entrenamiento de un modelo base para detección de semáforos
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
        data='data.yaml',                # Ruta al archivo con info del dataset
        epochs=50,                      # Número total de épocas (iteraciones completas)
        imgsz=160,                       # Tamaño de las imágenes de entrada (160x160)
        batch=32,                        # Tamaño del lote (cuántas imágenes procesa por paso)
        device=0,                        # Selección de GPU (0 = primera GPU)
        name='traffic_light_base_model' # Nombre de la carpeta de resultados
    )

# Punto de entrada del script
# freeze_support se requiere para compatibilidad en Windows; en Linux no afecta
if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
