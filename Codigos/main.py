<<<<<<< HEAD
import cv2
import numpy as np
import serial
import time

# --- CONFIGURACIÓN INICIAL ---
SERIAL_PORT = 'dev/ttyUSB0'  
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
=======
import cv2
import numpy as np
import serial
import time

# --- CONFIGURACIÓN ---
#SERIAL_PORT = 'COM5'  # <--- CAMBIA ESTO SI ES NECESARIO
#BAUD_RATE = 115200
# --- CONFIGURACIÓN ---
# En Linux suele ser /dev/ttyUSB0 o /dev/ttyACM0
SERIAL_PORT = '/dev/ttyUSB0'  
BAUD_RATE = 115200

# Conexión Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) # Espera a que el ESP32 reinicie
    print(f"Conectado a {SERIAL_PORT}")
except:
    print("ADVERTENCIA: No se pudo conectar al ESP32. Modo Simulación.")
    ser = None

# Variables PID (Ajustables luego con los sliders)
kp = 0.35
ki = 0.02
kd = 0.25

# Variables de estado
prev_error_x = 0
prev_error_y = 0
integral_x = 0
integral_y = 0

# Límite para evitar saturación del integrador (Anti-Windup)
LIMITE_INTEGRAL = 40 

def nothing(x): pass

# --- INTERFAZ GRÁFICA ---
cv2.namedWindow("Calibracion")

# 1. Trackbars de Color (Configurados para AZUL)
# Azul suele estar en Hue 100-125
cv2.createTrackbar("H Min", "Calibracion", 90, 179, nothing)
cv2.createTrackbar("H Max", "Calibracion", 130, 179, nothing)
cv2.createTrackbar("S Min", "Calibracion", 100, 255, nothing)
cv2.createTrackbar("S Max", "Calibracion", 255, 255, nothing)
cv2.createTrackbar("V Min", "Calibracion", 50, 255, nothing)
cv2.createTrackbar("V Max", "Calibracion", 255, 255, nothing)

# 2. Trackbars PID (Escalados x100)
cv2.createTrackbar("Kp", "Calibracion", int(kp*100), 500, nothing)
cv2.createTrackbar("Ki", "Calibracion", int(ki*100), 200, nothing)
cv2.createTrackbar("Kd", "Calibracion", int(kd*100), 500, nothing)

cap = cv2.VideoCapture(0, cv2.CAP_V4L2) # 0 es webcam interna, 1 externa

print("Sistema Iniciado. Presiona 'q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret: break

    # Opcional: Voltear imagen si se siente invertida
    # frame = cv2.flip(frame, 1) 

    # Dimensiones
    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2

    # --- 1. PROCESAMIENTO DE IMAGEN ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Leer Trackbars
    h_min = cv2.getTrackbarPos("H Min", "Calibracion")
    h_max = cv2.getTrackbarPos("H Max", "Calibracion")
    s_min = cv2.getTrackbarPos("S Min", "Calibracion")
    s_max = cv2.getTrackbarPos("S Max", "Calibracion")
    v_min = cv2.getTrackbarPos("V Min", "Calibracion")
    v_max = cv2.getTrackbarPos("V Max", "Calibracion")

    # Actualizar PID en tiempo real
    kp = cv2.getTrackbarPos("Kp", "Calibracion") / 100.0
    ki = cv2.getTrackbarPos("Ki", "Calibracion") / 100.0
    kd = cv2.getTrackbarPos("Kd", "Calibracion") / 100.0

    # Crear Máscara (Imagen Blanco y Negro)
    lower_blue = np.array([h_min, s_min, v_min])
    upper_blue = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Limpiar ruido (Erosión y Dilatación)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Encontrar contornos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    pid_x = 0
    pid_y = 0

    if len(contours) > 0:
        # Tomar el contorno más grande
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:
            # Dibujar tracking
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.line(frame, (center_x, center_y), (int(x), int(y)), (255, 0, 0), 2)

            # --- 2. CÁLCULO PID ---
            
            # Error X
            error_x = (center_x - x) / width * 100 # Invertido para corrección natural
            integral_x += error_x
            integral_x = np.clip(integral_x, -LIMITE_INTEGRAL, LIMITE_INTEGRAL) # Anti-windup
            derivative_x = error_x - prev_error_x
            pid_x = (kp * error_x) + (ki * integral_x) + (kd * derivative_x)
            prev_error_x = error_x

            # Error Y
            error_y = (center_y - y) / height * 100 # Invertido para corrección natural
            integral_y += error_y
            integral_y = np.clip(integral_y, -LIMITE_INTEGRAL, LIMITE_INTEGRAL)
            derivative_y = error_y - prev_error_y
            pid_y = (kp * error_y) + (ki * integral_y) + (kd * derivative_y)
            prev_error_y = error_y
    
    else:
        # Si pierde la pelota, resetea integrales para no acumular error fantasma
        integral_x = 0
        integral_y = 0

    # Dibujar centro
    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    # --- 3. COMUNICACIÓN SERIAL ---
    if ser is not None and ser.is_open:
        # Enviamos la corrección necesaria (float con 2 decimales)
        data = f"{pid_x:.2f},{pid_y:.2f}\n"
        ser.write(data.encode())

    # Mostrar ventanas
    cv2.imshow("Camara", frame)
    cv2.imshow("Mascara", mask) # <--- Aquí ves el filtro blanco/negro

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if ser:
    ser.write("0,0\n".encode()) # Reset al cerrar
    ser.close()
>>>>>>> Marcelo
cv2.destroyAllWindows()