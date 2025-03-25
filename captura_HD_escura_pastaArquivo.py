import cv2
from pyudev import Context
import time
import serial
import os

class PS3CameraController:
    def __init__(self, arduino_port='/dev/ttyACM0', baudrate=115200, fps=30, num_pairs=10):
        """
        Inicializa el controlador de la cámara PS3 Eye
        
        Args:
            arduino_port: Puerto del Arduino
            baudrate: Velocidad de comunicación
            fps: Frames por segundo
            num_pairs: Número de pares de imágenes (brillante/oscura) a capturar
        """
        self.fps = fps
        self.num_pairs = num_pairs
        # Configuración Arduino
        try:
            self.arduino = serial.Serial(arduino_port, baudrate, timeout=1)
            time.sleep(2)  # Esperar a que Arduino se reinicie
            self.configurar_parametros_iniciales()
        except Exception as e:
            print(f"Error conectando con Arduino: {e}")
            self.arduino = None

    def configurar_parametros_iniciales(self):
        """Configura los parámetros iniciales para el control de las luces estroboscópicas"""
        if self.arduino:
            # Periodo total del ciclo (1ms = 1000us)
            self.arduino.write(b'p1000\n')
            time.sleep(0.1)
            # Tiempo de encendido de los LEDs (80us)
            self.arduino.write(b'k80\n')
            time.sleep(0.1)
            # Delay antes del encendido (192us)
            self.arduino.write(b'h192\n')
            time.sleep(0.1)

    def obtener_dispositivo_por_identificador(self, id_vendor, id_model):
        """Encuentra la cámara PS3 Eye por su ID de vendor y modelo"""
        ctx = Context()
        for device in ctx.list_devices(subsystem='video4linux'):
            properties = device.properties
            if 'ID_VENDOR_ID' in properties and 'ID_MODEL_ID' in properties:
                if properties['ID_VENDOR_ID'] == id_vendor and properties['ID_MODEL_ID'] == id_model:
                    return device.device_node
        print("Dispositivo no encontrado")
        return None

    def crear_directorio_capturas(self):
        """Crea un directorio para almacenar las capturas con timestamp"""
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        dir_name = f"capturas_{timestamp}"
        os.makedirs(dir_name, exist_ok=True)
        return dir_name

    def capturar_imagenes(self):
        """Captura pares de imágenes alternando entre pupila brillante y oscura"""
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

        # Crear directorio para las capturas
        dir_capturas = self.crear_directorio_capturas()

        try:
            # Iniciar modo strobe
            self.arduino.write(b'V\n')
            response = self.arduino.readline().decode().strip()
            print(f"Modo strobe iniciado: {response}")
            
            intervalo = 1 / self.fps
            pares_capturados = 0
            ultimo_tiempo = time.time()

            while pares_capturados < self.num_pairs:
                tiempo_actual = time.time()
                
                if tiempo_actual - ultimo_tiempo >= intervalo:
                    # Capturar frame con pupila brillante
                    ret, frame_bright = cap.read()
                    if not ret:
                        print("Error al capturar frame brillante")
                        break

                    # Pequeña pausa para asegurar la alternancia de LEDs
                    time.sleep(0.01)

                    # Capturar frame con pupila oscura
                    ret, frame_dark = cap.read()
                    if not ret:
                        print("Error al capturar frame oscuro")
                        break

                    # Guardar las imágenes
                    cv2.imwrite(f"{dir_capturas}/bright_{pares_capturados}.png", frame_bright)
                    cv2.imwrite(f"{dir_capturas}/dark_{pares_capturados}.png", frame_dark)
                    print(f"Par de capturas {pares_capturados + 1} de {self.num_pairs}")

                    # Señal para Arduino
                    self.arduino.write(b'1\n')

                    pares_capturados += 1
                    ultimo_tiempo = tiempo_actual

        except Exception as e:
            print(f"Error durante la captura de imágenes: {e}")
        finally:
            # Detener modo strobe
            self.arduino.write(b's\n')
            cap.release()
            if self.arduino:
                self.arduino.close()

    def ajustar_parametros_strobe(self, periodo_us=1000, tiempo_encendido_us=80, delay_us=192):
        """Ajusta los parámetros de las luces estroboscópicas"""
        if self.arduino:
            self.arduino.write(f'p{periodo_us}\n'.encode())
            time.sleep(0.1)
            self.arduino.write(f'k{tiempo_encendido_us}\n'.encode())
            time.sleep(0.1)
            self.arduino.write(f'h{delay_us}\n'.encode())
            time.sleep(0.1)

if __name__ == "__main__":
    # Configuración para capturar 10 pares de imágenes a 5 FPS
    controlador = PS3CameraController(
        arduino_port='/dev/ttyACM0',
        fps=5,
        num_pairs=10
    )
    
    # Capturar imágenes
    controlador.capturar_imagenes()
