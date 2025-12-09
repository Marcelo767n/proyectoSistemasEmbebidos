<<<<<<< HEAD
import cv2
=======
import cv2 as cv
>>>>>>> Marcelo
import numpy as np
import serial
import time

<<<<<<< HEAD
# --- CONFIGURACIÓN INICIAL ---
SERIAL_PORT = 'COM5'  # <--- ¡CAMBIA ESTO POR TU PUERTO! (Ej: /dev/ttyUSB0 en Linux/Mac)
BAUD_RATE = 115200

# Inicializar comunicación Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) # Dar tiempo al ESP32 para reiniciarse
    print(f"Conectado a {SERIAL_PORT}")
except:
    print(f"ADVERTENCIA: No se pudo conectar a {SERIAL_PORT}. Modo solo visión.")
    ser = None

# Variables PID Globales (Valores iniciales)
kp = 0.4
ki = 0.0
kd = 0.2

# Variables de estado del PID
prev_error_x = 0
prev_error_y = 0
integral_x = 0
integral_y = 0

# Función vacía para los trackbars
def nothing(x):
    pass

# Crear ventana y trackbars para calibración
cv2.namedWindow("Calibracion")
# Trackbars para color (HSV)
cv2.createTrackbar("H Min", "Calibracion", 0, 179, nothing)
cv2.createTrackbar("H Max", "Calibracion", 25, 179, nothing) # Default naranja
cv2.createTrackbar("S Min", "Calibracion", 130, 255, nothing)
cv2.createTrackbar("S Max", "Calibracion", 255, 255, nothing)
cv2.createTrackbar("V Min", "Calibracion", 150, 255, nothing)
cv2.createTrackbar("V Max", "Calibracion", 255, 255, nothing)
# Trackbars para PID
cv2.createTrackbar("Kp", "Calibracion", int(kp*100), 200, nothing) # Escala x100
cv2.createTrackbar("Ki", "Calibracion", int(ki*100), 100, nothing)
cv2.createTrackbar("Kd", "Calibracion", int(kd*100), 200, nothing)

cap = cv2.VideoCapture(0) # 0 es usualmente la webcam integrada. Usa 1 si es externa.

def map_servo(value):
    """Mapea la salida del PID a angulos del servo (60-120 grados)"""
    # El centro es 90. Si el PID da +20, servo va a 110.
    angle = 90 + int(value)
    # Limites mecanicos (Safety)
    if angle > 120: angle = 120
    if angle < 60: angle = 60
    return angle

print("Presiona 'q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # 1. Preprocesamiento de imagen
    frame = cv2.flip(frame, 1) # Espejo horizontal (opcional, depende de tu camara)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2

    # Leer valores de los trackbars
    h_min = cv2.getTrackbarPos("H Min", "Calibracion")
    h_max = cv2.getTrackbarPos("H Max", "Calibracion")
    s_min = cv2.getTrackbarPos("S Min", "Calibracion")
    s_max = cv2.getTrackbarPos("S Max", "Calibracion")
    v_min = cv2.getTrackbarPos("V Min", "Calibracion")
    v_max = cv2.getTrackbarPos("V Max", "Calibracion")
    
    # Actualizar constantes PID desde trackbars (dividimos por 100 para tener decimales)
    kp = cv2.getTrackbarPos("Kp", "Calibracion") / 100.0
    ki = cv2.getTrackbarPos("Ki", "Calibracion") / 100.0
    kd = cv2.getTrackbarPos("Kd", "Calibracion") / 100.0

    # 2. Detección de la pelota
    lower_color = np.array([h_min, s_min, v_min])
    upper_color = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    # Limpieza de ruido (Erosion + Dilatacion)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    servo_x_angle = 90
    servo_y_angle = 90

    if len(contours) > 0:
        # Encontrar el contorno mas grande (la pelota)
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        
        if radius > 10: # Filtro para evitar ruidos pequeños
            # Dibujar la pelota detectada
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.line(frame, (center_x, center_y), (int(x), int(y)), (0, 0, 255), 2)

            # --- 3. CÁLCULO PID ---
            # Eje X
            error_x = (x - center_x) / width * 100 # Normalizar error
            integral_x += error_x
            derivative_x = error_x - prev_error_x
            output_x = (kp * error_x) + (ki * integral_x) + (kd * derivative_x)
            prev_error_x = error_x

            # Eje Y
            error_y = (y - center_y) / height * 100
            integral_y += error_y
            derivative_y = error_y - prev_error_y
            output_y = (kp * error_y) + (ki * integral_y) + (kd * derivative_y)
            prev_error_y = error_y
            
            # --- 4. MAPEO A SERVOS ---
            # NOTA: Dependiendo de como montes tus servos, puede que necesites
            # invertir la suma/resta:  90 - int(output_x)
            servo_x_angle = map_servo(-output_x) # Pruebe invirtiendo el signo si se aleja
            servo_y_angle = map_servo(output_y)  # Pruebe invirtiendo el signo si se aleja

    # Dibujar punto central (SetPoint)
    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

    # 5. ENVIAR A ESP32
    if ser is not None and ser.is_open:
        # Formato "X,Y\n"
        data_packet = f"{servo_x_angle},{servo_y_angle}\n"
        ser.write(data_packet.encode())

    # Mostrar imagen
    cv2.imshow("Camara", frame)
    cv2.imshow("Mascara", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if ser is not None:
    ser.close()
cv2.destroyAllWindows()
=======
# --- CONFIGURACIÓN SERIAL ---
# CAMBIA ESTO por el puerto donde conectaste el Arduino
# Windows: 'COM3', 'COM4', etc.
# Linux/Mac: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200 # Debe coincidir con el Serial.begin() de tu Arduino

# Coordenadas centrales (Deben coincidir con la lógica del Arduino)
center_x = 250
center_y = 250 

def main():
    print("Iniciando Visión Artificial (Modo Serial)...")
    
    # 1. Iniciar Comunicación Serial
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Conectado al puerto {SERIAL_PORT}")
        # Esperar 2 segundos a que el Arduino se reinicie al abrir puerto
        time.sleep(2) 
    except Exception as e:
        print(f"ERROR abriendo puerto serial: {e}")
        print("Verifica que el Arduino esté conectado y el puerto sea correcto.")
        return

    # 2. Iniciar Cámara
    cap = cv.VideoCapture(0)
    
    if not cap.isOpened():
        print("ERROR: No se pudo abrir la cámara.")
        return

    print("Presiona 'q' para salir.")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # --- LÓGICA DE VISIÓN (Misma que el repositorio original) ---
        
        # Convertir a HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Rango de color naranja (Ajusta estos valores si tu bola no se detecta)
        # Formato: [Hue, Saturation, Value]
        lower_value = np.array([0, 80, 100])
        higher_value = np.array([50, 255, 255])
        
        # Crear máscara
        mask = cv.inRange(hsv, lower_value, higher_value)
        mask = cv.blur(mask, (6, 6))
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        
        # Encontrar contornos
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        msg = "NODETECT\n"
        detected = False
        
        for cnt in contours:
            area = cv.contourArea(cnt)
            # Filtro de área para ignorar ruido pequeño
            if area > 1500:
                (x, y), radius = cv.minEnclosingCircle(cnt)
                radius = int(radius)
                
                if radius > 10:
                    x = int(x)
                    # El código original invierte Y: y = 480 - y
                    # Mantenemos esa lógica para compatibilidad
                    y_fisica = 480 - int(y) 
                    
                    # Dibujar en pantalla (Feedback visual)
                    cv.circle(frame, (x, int(y)), radius, (0, 255, 0), 2)
                    cv.circle(frame, (x, int(y)), 2, (0, 0, 255), 3)
                    
                    # Preparar mensaje "X,Y" con salto de línea
                    msg = f"{x},{y_fisica}\n"
                    detected = True
                    break # Solo tomamos la bola más grande
        
        # Enviar al Arduino por Serial
        try:
            if ser.is_open:
                ser.write(msg.encode('utf-8'))
                if detected:
                    print(f"Enviando: {msg.strip()}")
        except Exception as e:
            print(f"Error enviando datos serial: {e}")
            break

        # Mostrar video en ventanas
        cv.imshow("Vision Bola", frame)
        cv.imshow("Mascara (Blanco = Detectado)", mask)
        
        # Salir con tecla 'q'
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Limpieza al cerrar
    cap.release()
    cv.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
        print("Conexión Serial cerrada.")

if __name__ == "__main__":
    main()
>>>>>>> Marcelo
