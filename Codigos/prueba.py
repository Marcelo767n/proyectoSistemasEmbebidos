import cv2
import numpy as np
import serial
import time
import os

# --- CORRECCIÓN DE ERROR GRÁFICO (QT/WAYLAND) ---
# Esto evita el error "Could not find the Qt platform plugin wayland"
os.environ["QT_QPA_PLATFORM"] = "xcb"

# --- CONFIGURACIÓN ---
# En Linux, el ESP32 suele ser ttyUSB0. Si no conecta, prueba ttyACM0.
SERIAL_PORT = '/dev/ttyUSB0'  
BAUD_RATE = 115200

# TU CÁMARA EXTERNA (Identificada como índice 4)
CAMERA_INDEX = 4

# --- CONEXIÓN SERIAL ---
try:
    print(f"Intentando conectar a {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2) # Espera a que el ESP32 reinicie
    print(f"✅ Conectado exitosamente a {SERIAL_PORT}")
except Exception as e:
    print(f"⚠️ ADVERTENCIA: No se pudo conectar al ESP32. ({e})")
    print("   -> Modo Simulación (Solo visión).")
    ser = None

# --- VARIABLES PID ---
kp = 0.35
ki = 0.02
kd = 0.25

prev_error_x = 0
prev_error_y = 0
integral_x = 0
integral_y = 0
LIMITE_INTEGRAL = 40 

def nothing(x): pass

# --- INTERFAZ GRÁFICA ---
cv2.namedWindow("Calibracion")

# Trackbars de Color
cv2.createTrackbar("H Min", "Calibracion", 90, 179, nothing)
cv2.createTrackbar("H Max", "Calibracion", 130, 179, nothing)
cv2.createTrackbar("S Min", "Calibracion", 100, 255, nothing)
cv2.createTrackbar("S Max", "Calibracion", 255, 255, nothing)
cv2.createTrackbar("V Min", "Calibracion", 50, 255, nothing)
cv2.createTrackbar("V Max", "Calibracion", 255, 255, nothing)

# Trackbars PID
cv2.createTrackbar("Kp", "Calibracion", int(kp*100), 500, nothing)
cv2.createTrackbar("Ki", "Calibracion", int(ki*100), 200, nothing)
cv2.createTrackbar("Kd", "Calibracion", int(kd*100), 500, nothing)

# --- INICIALIZACIÓN CÁMARA ---
print(f"Abriendo cámara en índice {CAMERA_INDEX}...")
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

# Configuración para estabilidad (intentar apagar auto-foco)
# Si la cámara sigue re-enfocando, usa cinta adhesiva en el lente.
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
cap.set(cv2.CAP_PROP_FOCUS, 0) # 0 es infinito (o prueba valores hasta 255)

if not cap.isOpened():
    print("❌ ERROR CRÍTICO: No se puede abrir la cámara 4.")
    print("   -> Verifica que esté conectada.")
    exit()

print("Sistema Iniciado. Presiona 'q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret: 
        print("Error leyendo frame de la cámara")
        break

    # Dimensiones
    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2

    # --- 1. PROCESAMIENTO ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Leer Trackbars
    h_min = cv2.getTrackbarPos("H Min", "Calibracion")
    h_max = cv2.getTrackbarPos("H Max", "Calibracion")
    s_min = cv2.getTrackbarPos("S Min", "Calibracion")
    s_max = cv2.getTrackbarPos("S Max", "Calibracion")
    v_min = cv2.getTrackbarPos("V Min", "Calibracion")
    v_max = cv2.getTrackbarPos("V Max", "Calibracion")

    kp = cv2.getTrackbarPos("Kp", "Calibracion") / 100.0
    ki = cv2.getTrackbarPos("Ki", "Calibracion") / 100.0
    kd = cv2.getTrackbarPos("Kd", "Calibracion") / 100.0

    # Máscara
    lower_blue = np.array([h_min, s_min, v_min])
    upper_blue = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    pid_x = 0
    pid_y = 0

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.line(frame, (center_x, center_y), (int(x), int(y)), (255, 0, 0), 2)

            # --- 2. CÁLCULO PID ---
            error_x = (center_x - x) / width * 100
            integral_x += error_x
            integral_x = np.clip(integral_x, -LIMITE_INTEGRAL, LIMITE_INTEGRAL)
            derivative_x = error_x - prev_error_x
            pid_x = (kp * error_x) + (ki * integral_x) + (kd * derivative_x)
            prev_error_x = error_x

            error_y = (center_y - y) / height * 100
            integral_y += error_y
            integral_y = np.clip(integral_y, -LIMITE_INTEGRAL, LIMITE_INTEGRAL)
            derivative_y = error_y - prev_error_y
            pid_y = (kp * error_y) + (ki * integral_y) + (kd * derivative_y)
            prev_error_y = error_y
    else:
        integral_x = 0
        integral_y = 0

    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    # --- 3. COMUNICACIÓN SERIAL ---
    if ser is not None and ser.is_open:
        data = f"{pid_x:.2f},{pid_y:.2f}\n"
        try:
            ser.write(data.encode())
        except Exception as e:
            print(f"Error enviando datos: {e}")

    cv2.imshow("Camara Externa", frame)
    cv2.imshow("Mascara", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if ser:
    ser.write("0,0\n".encode())
    ser.close()
cap.release()
cv2.destroyAllWindows()