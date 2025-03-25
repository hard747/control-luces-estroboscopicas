import cv2
import numpy as np
import matplotlib.pyplot as plt

def segment_pupil(bright_path, dark_path, threshold=30):
    """
    Segmenta la pupila usando imágenes de pupila brillante y oscura.
    
    Args:
        bright_path: Ruta a la imagen de pupila brillante
        dark_path: Ruta a la imagen de pupila oscura
        threshold: Valor de umbral para la segmentación (0-255)
    
    Returns:
        Diccionario con imágenes de cada etapa del proceso
    """
    # Leer las imágenes en escala de grises
    bright_img = cv2.imread(bright_path, cv2.IMREAD_GRAYSCALE)
    dark_img = cv2.imread(dark_path, cv2.IMREAD_GRAYSCALE)
    
    if bright_img is None or dark_img is None:
        raise ValueError("No se pudieron cargar las imágenes correctamente.")
    
    if bright_img.shape != dark_img.shape:
        raise ValueError("Las imágenes deben tener el mismo tamaño.")
    
    # Calcular la diferencia entre imágenes
    diff = cv2.absdiff(bright_img, dark_img)
    
    # Visualizar el histograma de la diferencia
    plt.hist(diff.ravel(), bins=256, range=(0, 256))
    plt.title("Histograma de diferencias")
    plt.show()
    
    # Aplicar umbral
    _, pupil = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    
    # Mejorar la segmentación con operaciones morfológicas
    kernel = np.ones((5, 5), np.uint8)
    pupil_morph = cv2.morphologyEx(pupil, cv2.MORPH_OPEN, kernel)
    pupil_morph = cv2.morphologyEx(pupil_morph, cv2.MORPH_CLOSE, kernel)
    
    # Encontrar el contorno más grande
    contours, _ = cv2.findContours(pupil_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = np.zeros_like(pupil_morph)
    
    if len(contours) > 0:
        # Seleccionar el contorno más grande (se asume que es la pupila)
        largest_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(result, [largest_contour], -1, (255), -1)
    
    return {
        'bright': bright_img,
        'dark': dark_img,
        'difference': diff,
        'threshold': pupil,
        'morphology': pupil_morph,
        'final': result
    }

def main():
    # Rutas de las imágenes
    bright_path = "pupil_14.jpg"  # Imagen de pupila brillante
    dark_path = "pupil_13.jpg"      # Imagen de pupila oscura
    
    try:
        # Procesar las imágenes
        results = segment_pupil(bright_path, dark_path, threshold=30)
        
        # Mostrar imágenes de cada etapa
        for step, img in results.items():
            cv2.imshow(f"Paso: {step}", img)
        
        # Guardar las imágenes resultantes
        for name, img in results.items():
            cv2.imwrite(f'pupil_{name}.jpg', img)
        
        print("\nSegmentación completada y resultados guardados.")
        print("Presiona cualquier tecla para cerrar...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == "__main__":
    main()
