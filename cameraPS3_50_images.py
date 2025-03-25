import time
import cv2
import serial
from pyudev import Context

# Configurar puerto serie
ser = serial.Serial('/dev/ttyACM0', 115200)  # Cambia a tu puerto adecuado
time.sleep(2)  # Espera para establecer comunicación

# Función para obtener el dispositivo de la cámara por identificador
def obtener_dispositivo_por_identificador(id_vendor, id_model):
    ctx = Context()
    for device in ctx.list_devices(subsystem='video4linux'):
        properties = device.properties
        if 'ID_VENDOR_ID' in properties and 'ID_MODEL_ID' in properties:
            if properties['ID_VENDOR_ID'] == id_vendor and properties['ID_MODEL_ID'] == id_model:
                return device.device_node

# Definir los nombres de archivo para guardar las imágenes
#dark_pupil_image = "pupil_{}.jpg"
bright_pupil_image = "pupil_{}.jpg"

# Configuración para la cámara
id_vendor = '1415'  # Cambia con el ID de tu cámara
id_model = '2000'   # Cambia con el ID de tu cámara
fps = 60             # Frames por segundo
cantidad_fotos = 60 # Número de imágenes a capturar

# Obtener el dispositivo de la cámara
dispositivo = obtener_dispositivo_por_identificador(id_vendor, id_model)

if dispositivo:
    cap = cv2.VideoCapture(dispositivo)

    if not cap.isOpened():
        print(f"No se pudo abrir el dispositivo")
    else:
        try:
            # Ajusta el FPS de la cámara
            cap.set(cv2.CAP_PROP_FPS, fps)

            # Enviar señal de inicio al Arduino para activar el control de LEDs
            ser.write(b"1")  # Enviar '1' para iniciar la sincronización
            time.sleep(0.1)  # Espera para asegurar que el Arduino recibe la señal

            for i in range(cantidad_fotos):
                # Capturar la imagen
                ret, frame = cap.read()

                if not ret:
                    print("No se pudo capturar el frame")
                    break

                # Guardar la imagen con el nombre adecuado
                if i % 2 == 0:
                    cv2.imwrite(bright_pupil_image.format(i), frame)  # Guardar imagen de pupila oscura
                else:
                    cv2.imwrite(bright_pupil_image.format(i), frame)  # Guardar imagen de pupila brillante

                # Pausa entre capturas para sincronizar con la frecuencia de la cámara
                time.sleep(1 / fps)

            # Enviar señal de finalización al Arduino para apagar las luces
            ser.write(b"0")  # Enviar '0' para detener la sincronización
            print("Sincronización con Arduino finalizada, luces apagadas.")

            cap.release()

        except KeyboardInterrupt:
            print("Captura interrumpida.")
            ser.write(b"0")  # Asegura que las luces se apaguen si se interrumpe
        finally:
            ser.close()
else:
    print("No se encontró ningún dispositivo con el identificador especificado.")
