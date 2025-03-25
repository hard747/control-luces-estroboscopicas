import cv2
from pyudev import Context
import time
import serial

class PS3CameraController:
    def __init__(self, arduino_port='/dev/ttyACM0', baudrate=115200):
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
            # Configura el período base (en microsegundos)
            self.arduino.write(b'p1000\n')  # 1000 microsegundos = 1ms
            time.sleep(0.1)
            
            # Configura el tiempo de encendido del LED (en microsegundos)
            self.arduino.write(b'k80\n')    # 80 microsegundos de pulso
            time.sleep(0.1)
            
            # Configura el delay después de la señal de la cámara
            self.arduino.write(b'h192\n')   # 192 microsegundos de delay
            time.sleep(0.1)

    def obtener_dispositivo_por_identificador(self, id_vendor, id_model):
        ctx = Context()
        for device in ctx.list_devices(subsystem='video4linux'):
            properties = device.properties
            if 'ID_VENDOR_ID' in properties and 'ID_MODEL_ID' in properties:
                if properties['ID_VENDOR_ID'] == id_vendor and properties['ID_MODEL_ID'] == id_model:
                    return device.device_node

    def iniciar_modo_strobe(self):
        """Inicia el modo de captura con luces estroboscópicas"""
        if self.arduino:
            # Envía el comando 'V' para iniciar el modo strobe
            self.arduino.write(b'V\n')
            response = self.arduino.readline().decode().strip()
            print(f"Modo strobe iniciado: {response}")

    def detener_modo_strobe(self):
        """Detiene el modo de captura con luces estroboscópicas"""
        if self.arduino:
            # Envía el comando 's' para detener el modo strobe
            self.arduino.write(b's\n')

    def capturar_imagenes(self, fps=30, cantidad_fotos=10):
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

        try:
            # Iniciar modo strobe
            self.iniciar_modo_strobe()
            
            intervalo = 1 / fps
            fotos_tomadas = 0
            ultimo_tiempo = time.time()

            while fotos_tomadas < cantidad_fotos:
                tiempo_actual = time.time()
                
                if tiempo_actual - ultimo_tiempo >= intervalo:
                    ret, frame = cap.read()
                    if not ret:
                        print("Error al capturar frame")
                        break

                    # Guardar la imagen capturada
                    cv2.imwrite(f"captura_{fotos_tomadas}.png", frame)
                    print(f"Captura {fotos_tomadas + 1} de {cantidad_fotos}")
                    
                    fotos_tomadas += 1
                    ultimo_tiempo = tiempo_actual

        finally:
            # Asegurarse de detener el modo strobe al terminar
            self.detener_modo_strobe()
            cap.release()

    def ajustar_parametros_strobe(self, periodo_us=1000, tiempo_encendido_us=80, delay_us=192):
        """
        Ajusta los parámetros de las luces estroboscópicas
        
        Args:
            periodo_us: Período en microsegundos entre pulsos
            tiempo_encendido_us: Duración del pulso de luz en microsegundos
            delay_us: Delay después de la señal de la cámara en microsegundos
        """
        if self.arduino:
            self.arduino.write(f'p{periodo_us}\n'.encode())
            time.sleep(0.1)
            self.arduino.write(f'k{tiempo_encendido_us}\n'.encode())
            time.sleep(0.1)
            self.arduino.write(f'h{delay_us}\n'.encode())
            time.sleep(0.1)

    def __del__(self):
        if self.arduino:
            self.detener_modo_strobe()
            self.arduino.close()

if __name__ == "__main__":
    # Ejemplo de uso
    controlador = PS3CameraController(arduino_port='/dev/ttyACM0')  # Ajusta el puerto según tu sistema
    
    # Opcional: ajustar parámetros personalizados
    # controlador.ajustar_parametros_strobe(periodo_us=2000, tiempo_encendido_us=100, delay_us=200)
    
    # Capturar imágenes
    controlador.capturar_imagenes(fps=5, cantidad_fotos=20)
