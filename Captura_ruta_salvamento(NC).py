import cv2
from pyudev import Context
import time
import serial
import os
from pathlib import Path

class PS3CameraController:
    def __init__(self, arduino_port='/dev/ttyACM0', baudrate=115200, fps=30, num_pairs=10):
        """
        Inicializa el controlador de la cámara PS3 Eye
        """
        self.fps = fps
        self.num_pairs = num_pairs
        # Obtener el directorio home del usuario
        self.base_dir = str(Path.home() / "ps3_eye_captures")
        # Crear el directorio base si no existe
        os.makedirs(self.base_dir, exist_ok=True)
        print(f"Las imágenes se guardarán en: {self.base_dir}")
        
        try:
            self.arduino = serial.Serial(arduino_port, baudrate, timeout=1)
            time.sleep(2)  # Esperar a que Arduino se reinicie
            print("Arduino conectado exitosamente")
        except Exception as e:
            print(f"Error conectando con Arduino: {e}")
            self.arduino = None

    def obtener_dispositivo_por_identificador(self, id_vendor, id_model):
        """Encuentra la cámara PS3 Eye por su ID de vendor y modelo"""
        ctx = Context()
        for device in ctx.list_devices(subsystem='video4linux'):
            properties = device.properties
            if 'ID_VENDOR_ID' in properties and 'ID_MODEL_ID' in properties:
                if properties['ID_VENDOR_ID'] == id_vendor and properties['ID_MODEL_ID'] == id_model:
                    return device.device_node
        return None

    def crear_directorio_capturas(self):
        """Crea un directorio para almacenar las capturas con timestamp"""
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        dir_name = os.path.join(self.base_dir, f"capturas_{timestamp}")
        os.makedirs(dir_name, exist_ok=True)
        print(f"Directorio de capturas creado: {dir_name}")
        return dir_name

    def verificar_guardado(self, ruta_archivo):
        """Verifica que la imagen se haya guardado correctamente"""
        if os.path.exists(ruta_archivo):
            tamaño = os.path.getsize(ruta_archivo)
            if tamaño > 0:
                print(f"Imagen guardada correctamente: {ruta_archivo} ({tamaño} bytes)")
                return True
            else:
                print(f"ERROR: La imagen se creó pero está vacía: {ruta_archivo}")
                return False
        else:
            print(f"ERROR: No se pudo guardar la imagen: {ruta_archivo}")
            return False

    def capturar_imagenes(self):
        """Captura imágenes alternando entre pupila brillante y oscura"""
        id_vendor = '1415'  # ID vendor para PS3 Eye
        id_model = '2000'
        dispositivo = self.obtener_dispositivo_por_identificador(id_vendor, id_model)

        if not dispositivo:
            print("No se encontró la cámara PS3")
            return

        cap = cv2.VideoCapture(dispositivo)
        if not cap.isOpened():
            print("No se pudo abrir la cámara PS3")
            return

        # Configurar la cámara
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Crear directorio para las capturas
        dir_capturas = self.crear_directorio_capturas()
        
        # Crear subdirectorios para bright y dark
        bright_dir = os.path.join(dir_capturas, "bright")
        dark_dir = os.path.join(dir_capturas, "dark")
        os.makedirs(bright_dir, exist_ok=True)
        os.makedirs(dark_dir, exist_ok=True)
        print(f"Subdirectorios creados:\n  Bright: {bright_dir}\n  Dark: {dark_dir}")

        try:
            # Iniciar modo strobe
            self.arduino.write(b'V\n')
            print("Iniciando captura de imágenes...")
            
            capturas = 0
            ultimo_tiempo = time.time()
            intervalo = 1.0 / (self.fps / 2)  # Ajustado para dar tiempo a la alternancia

            while capturas < self.num_pairs * 2:
                if time.time() - ultimo_tiempo >= intervalo:
                    ret, frame = cap.read()
                    if not ret:
                        print("Error al capturar frame")
                        break

                    # Determinar tipo de imagen y directorio
                    es_bright = capturas % 2 == 0
                    tipo = 'bright' if es_bright else 'dark'
                    dir_actual = bright_dir if es_bright else dark_dir
                    num_par = capturas // 2
                    
                    # Construir ruta completa
                    nombre_archivo = f"{tipo}_{num_par:03d}.png"
                    ruta_completa = os.path.join(dir_actual, nombre_archivo)
                    
                    # Guardar imagen
                    cv2.imwrite(ruta_completa, frame)
                    
                    # Verificar que se guardó correctamente
                    if self.verificar_guardado(ruta_completa):
                        print(f"Captura {capturas + 1} de {self.num_pairs * 2} ({tipo})")
                        capturas += 1
                        ultimo_tiempo = time.time()
                    else:
                        print("Reintentando captura...")

        except Exception as e:
            print(f"Error durante la captura: {e}")
        finally:
            # Detener modo strobe
            self.arduino.write(b's\n')
            cap.release()
            if self.arduino:
                self.arduino.close()
            print(f"\nCaptura finalizada")
            print(f"Las imágenes se guardaron en: {dir_capturas}")
            print(f"Total de imágenes capturadas: {capturas}")
            # Mostrar contenido del directorio
            print("\nContenido del directorio de capturas:")
            print("Imágenes bright:")
            for f in sorted(os.listdir(bright_dir)):
                print(f"  {f}")
            print("Imágenes dark:")
            for f in sorted(os.listdir(dark_dir)):
                print(f"  {f}")

if __name__ == "__main__":
    controlador = PS3CameraController(
        arduino_port='/dev/ttyACM0',
        fps=10,  # Reducido para mejor sincronización
        num_pairs=10  # 10 pares de imágenes (bright/dark)
    )
    controlador.capturar_imagenes()
