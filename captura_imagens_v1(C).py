import cv2
from pyudev import Context
import time
import serial
import os

class PS3CameraController:
    # Definición de modos de iluminación
    MODO_OFF_AXIS = 1    # Strobo externo
    MODO_ALL = 2         # Todos los strobos
    MODO_ON_AXIS = 3     # Strobo interno
    MODO_OFF = 4         # Strobos apagados

    def __init__(self, arduino_port='/dev/ttyACM0', baudrate=115200, fps=60, num_pairs=10):
        """
        Inicializa el controlador de la cámara PS3 Eye
        """
        self.fps = fps
        self.num_pairs = num_pairs
        self.pin_config = {
            'PIN1': 1,    # Pin de entrada para strobo todas/interno
            'PIN2': 2,    # Pin de entrada para strobo externo/apagado
            'PIN_A4': 18, # Pin de salida para strobo externo/todas
            'PIN_A5': 19  # Pin de salida para strobo interno/apagado
        }
        try:
            self.arduino = serial.Serial(arduino_port, baudrate, timeout=1)
            time.sleep(2)  # Esperar a que Arduino se reinicie
            self._configurar_pines()
            print("Arduino conectado exitosamente")
        except Exception as e:
            print(f"Error conectando con Arduino: {e}")
            self.arduino = None

    def _configurar_pines(self):
        """Configura los pines del Arduino para los diferentes modos de strobo"""
        if self.arduino:
            # Configurar pines como salidas
            self.arduino.write(f"M{self.pin_config['PIN_A4']}O\n".encode())  # A4 como salida
            self.arduino.write(f"M{self.pin_config['PIN_A5']}O\n".encode())  # A5 como salida
            # Configurar pines como entradas
            self.arduino.write(f"M{self.pin_config['PIN1']}I\n".encode())    # PIN1 como entrada
            self.arduino.write(f"M{self.pin_config['PIN2']}I\n".encode())    # PIN2 como entrada

    def set_modo_strobo(self, modo):
        """
        Configura el modo de iluminación estroboscópica
        """
        if not self.arduino:
            return False

        if modo == self.MODO_OFF_AXIS:
            # Strobo externo (off-axis): PIN2 + A4
            self.arduino.write(f"H{self.pin_config['PIN_A4']}\n".encode())  # A4 HIGH
            self.arduino.write(f"H{self.pin_config['PIN2']}\n".encode())    # PIN2 HIGH
            self.arduino.write(f"L{self.pin_config['PIN_A5']}\n".encode())  # A5 LOW
            self.arduino.write(f"L{self.pin_config['PIN1']}\n".encode())    # PIN1 LOW
        
        elif modo == self.MODO_ALL:
            # Todos los strobos: PIN1 + A4
            self.arduino.write(f"H{self.pin_config['PIN_A4']}\n".encode())  # A4 HIGH
            self.arduino.write(f"H{self.pin_config['PIN1']}\n".encode())    # PIN1 HIGH
            self.arduino.write(f"L{self.pin_config['PIN_A5']}\n".encode())  # A5 LOW
            self.arduino.write(f"L{self.pin_config['PIN2']}\n".encode())    # PIN2 LOW
        
        elif modo == self.MODO_ON_AXIS:
            # Strobo interno (on-axis): PIN1 + A5
            self.arduino.write(f"H{self.pin_config['PIN_A5']}\n".encode())  # A5 HIGH
            self.arduino.write(f"H{self.pin_config['PIN1']}\n".encode())    # PIN1 HIGH
            self.arduino.write(f"L{self.pin_config['PIN_A4']}\n".encode())  # A4 LOW
            self.arduino.write(f"L{self.pin_config['PIN2']}\n".encode())    # PIN2 LOW
        
        elif modo == self.MODO_OFF:
            # Strobos apagados: PIN2 + A5
            self.arduino.write(f"H{self.pin_config['PIN_A5']}\n".encode())  # A5 HIGH
            self.arduino.write(f"H{self.pin_config['PIN2']}\n".encode())    # PIN2 HIGH
            self.arduino.write(f"L{self.pin_config['PIN_A4']}\n".encode())  # A4 LOW
            self.arduino.write(f"L{self.pin_config['PIN1']}\n".encode())    # PIN1 LOW
        
        time.sleep(0.01)  # Pequeña pausa para asegurar que los cambios se apliquen
        return True

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
        """Crea un directorio para almacenar las capturas"""
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        dir_name = f"capturas_{timestamp}"
        os.makedirs(dir_name, exist_ok=True)
        return dir_name

    def capturar_imagenes(self):
        """Captura imágenes alternando entre diferentes modos de iluminación"""
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

        try:
            print("Iniciando captura de imágenes...")
            capturas = 0
            ultimo_tiempo = time.time()
            intervalo = 1.0 / (self.fps / 2)  # Ajustado para dar tiempo a la alternancia

            while capturas < self.num_pairs * 2:
                if time.time() - ultimo_tiempo >= intervalo:
                    # Alternar entre modos de iluminación
                    if capturas % 2 == 0:
                        self.set_modo_strobo(self.MODO_ON_AXIS)  # Strobo interno para bright
                        tipo = 'bright'
                    else:
                        self.set_modo_strobo(self.MODO_OFF)      # Strobos apagados para dark
                        tipo = 'dark'

                    # Pequeña pausa para asegurar que el strobo se establezca
                    time.sleep(0.01)

                    ret, frame = cap.read()
                    if not ret:
                        print("Error al capturar frame")
                        break

                    # Guardar imagen
                    num_par = capturas // 2
                    nombre_archivo = f"{dir_capturas}/{tipo}_{num_par}.png"
                    cv2.imwrite(nombre_archivo, frame)
                    
                    print(f"Captura {capturas + 1} de {self.num_pairs * 2} ({tipo})")
                    
                    capturas += 1
                    ultimo_tiempo = time.time()

        except Exception as e:
            print(f"Error durante la captura: {e}")
        finally:
            # Apagar todos los strobos
            self.set_modo_strobo(self.MODO_OFF)
            cap.release()
            if self.arduino:
                self.arduino.close()
            print("Captura finalizada")

if __name__ == "__main__":
    controlador = PS3CameraController(
        arduino_port='/dev/ttyACM0',
        fps=10,  # Reducido para mejor sincronización
        num_pairs=10  # 10 pares de imágenes (bright/dark)
    )
    controlador.capturar_imagenes()
