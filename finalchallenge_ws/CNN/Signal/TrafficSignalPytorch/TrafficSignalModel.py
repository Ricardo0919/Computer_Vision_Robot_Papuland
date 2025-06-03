# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Reentrenamiento del modelo para detección 
#                                       de señales de tráfico
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
    model = YOLO('models/best.pt')
    #model = YOLO('yolov8s.pt')
    
    # Inicio del proceso de entrenamiento
    model.train(
        data='data.yaml',                    # Configuración del dataset
        epochs=60,                           # Número de épocas
        imgsz=160,                           # Tamaño de las imágenes
        batch=32,                            # Tamaño del lote
        device=0,                            # GPU
        name='traffic_signal_finetuned_aug', # Nombre de la corrida
        augment=True,                        # Activar data augmentation
        degrees=10,                          # Rotación aleatoria ±10°
        scale=0.5,                           # Escalado aleatorio (0.5 a 1.5)
        perspective=0.001,                   # Distorsión de perspectiva leve
        hsv_h=0.015,                         # Variación de tono
        hsv_s=0.7,                           # Variación de saturación
        hsv_v=0.4                            # Variación de brillo
    )

# Punto de entrada del script
# freeze_support se requiere para compatibilidad en Windows; en Linux no afecta
if __name__ == '__main__':
    multiprocessing.freeze_support()
    main()
