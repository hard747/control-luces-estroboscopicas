import cv2
import numpy as np
import time
import serial
from pyudev import Context

def find_pupil_centers(frame1, frame2):
    """
    Encuentra los centros de las pupilas en la diferencia entre dos frames, utilizando el centro de masa de blobs.
    """
    # Convertir a escala de grises y calcular la diferencia
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray1, gray2)

    # Aplicar filtro de mediana para reducir ruido
    diff = cv2.medianBlur(diff, 5)

    # Mejorar el contraste utilizando Otsu para un umbral dinámico
    _, thresh = cv2.threshold(diff, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Operaciones morfológicas para mejorar la detección
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.dilate(thresh, kernel, iterations=2)
    thresh = cv2.erode(thresh, kernel, iterations=1)

    # Encontrar componentes conectadas (blobs) y sus estadísticas
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresh)

    valid_centers = []
    for i in range(1, num_labels):  # El primer label (0) es el fondo, por lo que empezamos desde 1
        x, y, w, h, area = stats[i]
        cX, cY = centroids[i]

        if min_area < area < max_area:
            valid_centers.append((int(cX), int(cY), area))

    # Si se detectan 2 pupilas, ordenar por la coordenada x
    if len(valid_centers) == 2:
        valid_centers.sort(key=lambda x: x[0])
        return [(c[0], c[1]) for c in valid_centers], thresh
    elif len(valid_centers) == 1:  # Si solo hay una pupila
        return [(valid_centers[0][0], valid_centers[0][1])], thresh
    return None, None


def obtener_dispositivo_por_identificador(id_vendor, id_model):
    ctx = Context()
    for device in ctx.list_devices(subsystem='video4linux'):
        properties = device.properties
        if 'ID_VENDOR_ID' in properties and 'ID_MODEL_ID' in properties:
            if properties['ID_VENDOR_ID'] == id_vendor and properties['ID_MODEL_ID'] == id_model:
                return device.device_node
    return None


# Configuración inicial
min_area = 30
max_area = 800
id_vendor = '1415'
#id_vendor = '8086'
id_model = '2000'
#id_model = '0a66'
fps = 60

# Configurar puerto serie para Arduino
try:
    ser = serial.Serial('/dev/ttyACM0', 115200)
    time.sleep(2)
except serial.SerialException:
    print("Error al conectar con Arduino")
    exit()

# Inicializar la cámara
dispositivo = obtener_dispositivo_por_identificador(id_vendor, id_model)
if not dispositivo:
    print("No se encontró la cámara especificada")
    ser.close()
    exit()

cap = cv2.VideoCapture(dispositivo)
if not cap.isOpened():
    print("No se pudo abrir la cámara")
    ser.close()
    exit()

# Configurar FPS y resolución de la cámara
cap.set(cv2.CAP_PROP_FPS, fps)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Proceso en tiempo real
try:
    ser.write(b"1")  # Encender luces estroboscópicas
    time.sleep(0.1)

    prev_frame = None
    binary_image = None  # Aseguramos que la variable siempre esté definida

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error al capturar frame")
            break

        if prev_frame is not None:
            # Encontrar centros de pupilas y la imagen binaria
            pupil_centers, binary_image = find_pupil_centers(prev_frame, frame)

            if pupil_centers:
                # Pintar las pupilas con color verde fosforescente
                for (cX, cY) in pupil_centers:
                    cv2.circle(frame, (cX, cY), 5, (0, 255, 0), 2)  # Verde fosforescente para las pupilas

        # Mostrar la imagen original con las pupilas
        cv2.imshow('Pupil Detection', frame)

        # Mostrar la imagen binaria en tiempo real (si está disponible)
        if binary_image is not None:
            cv2.imshow('Binary Image', binary_image)

        # Esperar por una tecla para salir
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('0'):  # Desactivar luces si se presiona 'q' o '0'
            ser.write(b"0")  # Enviar comando para desactivar las luces estroboscópicas
            break

        prev_frame = frame

except KeyboardInterrupt:
    print("Interrupción del programa")

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
ser.close()
