import cv2
import numpy as np

# Cargar la imagen en blanco y negro
image = cv2.imread('olhopupil-modified.png', cv2.IMREAD_GRAYSCALE)

# Aplicar un umbral para detectar la pupila
_, thresholded = cv2.threshold(image, 50, 255, cv2.THRESH_BINARY_INV)

# Encontrar contornos
contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Dibujar la pupila brillante
for contour in contours:
    if cv2.contourArea(contour) > 100:  # Ajusta este valor según el tamaño de la pupila
        cv2.drawContours(image, [contour], -1, (255), thickness=cv2.FILLED)

# Guardar la imagen resultante
cv2.imwrite('olhopupil_resultante.jpg', image)
