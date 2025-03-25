import cv2
import numpy as np
import matplotlib.pyplot as plt

# Cargar las imágenes en color
imagen_brilhante = cv2.imread('pupila_brilhante.jpg')
imagen_oscura = cv2.imread('pupila_oscura.jpg')

# Verificar si las imágenes se cargaron correctamente
if imagen_brilhante is None:
    print("No se pudo cargar la imagen pupila brillante")
    exit()

if imagen_oscura is None:
    print("No se pudo cargar la imagen pupila oscura")
    exit()

# Convertir las imágenes a escala de grises
imagen_brilhante_gray = cv2.cvtColor(imagen_brilhante, cv2.COLOR_BGR2GRAY)
imagen_oscura_gray = cv2.cvtColor(imagen_oscura, cv2.COLOR_BGR2GRAY)

# Asegúrate de que las imágenes tengan el mismo tamaño
if imagen_brilhante_gray.shape != imagen_oscura_gray.shape:
    print("Las imágenes deben tener el mismo tamaño")
    exit()

# Calcular la diferencia entre las imágenes en escala de grises
diferencia = cv2.absdiff(imagen_brilhante_gray, imagen_oscura_gray)

# Aplicar un umbral para segmentar la pupila
_, umbral = cv2.threshold(diferencia, 30, 255, cv2.THRESH_BINARY)

# Opcional: realizar una operación de cierre para eliminar pequeñas imperfecciones
kernel = np.ones((5, 5), np.uint8)
umbral = cv2.morphologyEx(umbral, cv2.MORPH_CLOSE, kernel)

# Mostrar la imagen segmentada con la pupila
plt.imshow(umbral, cmap='gray')
plt.title("Pupila Segmentada")
plt.axis('off')
plt.show()

# Guardar la imagen segmentada
cv2.imwrite('pupila_segmentada.jpg', umbral)
