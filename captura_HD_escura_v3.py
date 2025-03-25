import cv2
from pyudev import Context
import time

def obtener_dispositivo_por_identificador(id_vendor, id_model):
    ctx = Context()
    for device in ctx.list_devices(subsystem='video4linux'):
        properties = device.properties
        if 'ID_VENDOR_ID' in properties and 'ID_MODEL_ID' in properties:
            if properties['ID_VENDOR_ID'] == id_vendor and properties['ID_MODEL_ID'] == id_model:
                return device.device_node

def capturar_imagenes(fps=30, cantidad_fotos=10):
    id_vendor = '1415'
    id_model = '2000'
    dispositivo = obtener_dispositivo_por_identificador(id_vendor, id_model)

    if dispositivo:
        cap = cv2.VideoCapture(dispositivo)

        if not cap.isOpened():
            print(f"No se pudo abrir el dispositivo")
            return

        intervalo = 1 / fps
        fotos_tomadas = 0

        while fotos_tomadas < cantidad_fotos:
            ret, frame = cap.read()

            if not ret:
                print("No se pudo capturar el frame")
                break

            cv2.imwrite(f"captura_{fotos_tomadas}.png", frame)
            fotos_tomadas += 1

            time.sleep(intervalo)

        cap.release()
    else:
        print(f"No se encontró ningún dispositivo con el identificador")

if __name__ == "__main__":
    # Ajusta los valores de fps y cantidad_fotos según tus necesidades
    capturar_imagenes(fps=120, cantidad_fotos=20)

