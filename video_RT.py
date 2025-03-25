import cv2
import numpy as np
import time
import serial
from pyudev import Context


def find_pupil_centers(frame1, frame2):
    """
    Encuentra los centros de las pupilas en la diferencia entre dos frames, utilizando blobs.
    """
    # Convertir a escala de grises y calcular la diferencia
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    diff = cv2.subtract(gray1, gray2)

    # Mejorar el contraste
    diff = cv2.equalizeHist(diff)

    # Aplicar umbralización
    _, thresh = cv2.threshold(diff, 80, 255, cv2.THRESH_BINARY)

    # Operaciones morfológicas para mejorar la detección
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv2.dilate(thresh, kernel, iterations=2)
    thresh = cv2.erode(thresh, kernel, iterations=1)

    # Encontrar contornos
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filtrar contornos
    valid_centers = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area < area < max_area:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                valid_centers.append((cX, cY, area))

    # Ordenar por área y tomar los dos más grandes
    valid_centers.sort(key=lambda x: x[2], reverse=True)
    valid_centers = valid_centers[:2]

    if len(valid_centers) == 2:
        # Ordenar por posición x para izquierda y derecha
        valid_centers.sort(key=lambda x: x[0])
        return [(c[0], c[1]) for c in valid_centers], thresh
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
id_model = '2000'
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
                # Visualizar resultados con colores diferenciados para pupilas
                left_pupil, right_pupil = pupil_centers
                cv2.circle(frame, left_pupil, 5, (0, 0, 255), 2)  # Rojo para la pupila izquierda
                cv2.circle(frame, right_pupil, 5, (255, 0, 0), 2)  # Azul para la pupila derecha

        # Mostrar la imagen original con las pupilas
        cv2.imshow('Pupil Detection', frame)

        # Mostrar la imagen binaria en tiempo real (si existe)
        if binary_image is not None:
            cv2.imshow('Binary Image', binary_image)

        # Salir con la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        prev_frame = frame.copy()
        time.sleep(1 / fps)

except KeyboardInterrupt:
    print("Interrupción manual")
finally:
    ser.write(b"0")  # Apagar luces estroboscópicas
    time.sleep(0.1)
    ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print("Proceso finalizado")
