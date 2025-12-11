import cv2
import numpy as np
import serial
import time
import os

# --- CORRECCIÓN DE ERROR GRÁFICO (LINUX/WAYLAND) ---
os.environ["QT_QPA_PLATFORM"] = "xcb"

# --- CONFIGURACIÓN SERIAL ---
SERIAL_PORT = '/dev/ttyUSB0'  
BAUD_RATE = 115200

# --- VARIABLES PID ---
kp_default = 0.35
ki_default = 0.02
kd_default = 0.25

# Memoria del PID (Variables de estado)
prev_error_x = 0
prev_error_y = 0

# ### NUEVO: Inicializamos la suma de errores ###
integral_x = 0
integral_y = 0

# ### NUEVO: Límite de seguridad ###
LIMITE_INTEGRAL = 1000 

def nothing(x): pass

# --- FUNCIÓN: BÚSQUEDA INTELIGENTE DE CÁMARA ---
def iniciar_camara_automatica():
    indices_a_probar = [0, 1, 2, 4, 6, 8] 
    print("\n--- BUSCANDO CÁMARA DISPONIBLE ---")
    for indice in indices_a_probar:
        print(f"🔍 Probando índice {indice}...", end=" ")
        cap = cv2.VideoCapture(indice, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print("✅ ¡CONECTADA!")
                try:
                    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                except: pass
                return cap
            cap.release()
    print("\n❌ ERROR CRÍTICO: No se encontró ninguna cámara.")
    return None

# --- 1. INICIALIZACIÓN DE HARDWARE ---
try:
    print(f"Intentando conectar al ESP32 en {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)
    print("✅ Serial Conectado.")
except Exception as e:
    print(f"⚠️ ADVERTENCIA: Sin conexión Serial ({e})")
    ser = None

cap = iniciar_camara_automatica()
if cap is None: exit() 

# --- 2. INTERFAZ GRÁFICA ---
cv2.namedWindow("Calibracion")

# Trackbars de Color
cv2.createTrackbar("H Min", "Calibracion", 90, 179, nothing)
cv2.createTrackbar("H Max", "Calibracion", 130, 179, nothing)
cv2.createTrackbar("S Min", "Calibracion", 150, 255, nothing)
cv2.createTrackbar("S Max", "Calibracion", 255, 255, nothing)
cv2.createTrackbar("V Min", "Calibracion", 150, 255, nothing)
cv2.createTrackbar("V Max", "Calibracion", 255, 255, nothing)

# Trackbars PID (Escalados x100)
# ### NUEVO: Ki se divide por 1000 para mayor precisión ###
cv2.createTrackbar("Kp", "Calibracion", int(kp_default*100), 500, nothing)
cv2.createTrackbar("Ki", "Calibracion", int(ki_default*1000), 100, nothing) 
cv2.createTrackbar("Kd", "Calibracion", int(kd_default*100), 500, nothing)

print("\nSistema Iniciado. Presiona 'q' para salir.")

# --- 3. BUCLE PRINCIPAL ---
while True:
    ret, frame = cap.read()
    if not ret: break

    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2

    # Procesamiento
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Leer sliders
    h_min = cv2.getTrackbarPos("H Min", "Calibracion")
    h_max = cv2.getTrackbarPos("H Max", "Calibracion")
    s_min = cv2.getTrackbarPos("S Min", "Calibracion")
    s_max = cv2.getTrackbarPos("S Max", "Calibracion")
    v_min = cv2.getTrackbarPos("V Min", "Calibracion")
    v_max = cv2.getTrackbarPos("V Max", "Calibracion")

    # Leer PID
    kp = cv2.getTrackbarPos("Kp", "Calibracion") / 100.0
    ki = cv2.getTrackbarPos("Ki", "Calibracion") / 1000.0 # ### NUEVO: DIVIDIDO POR 1000 ###
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

            # --- CÁLCULO PID ---
            
            # === EJE X ===
            error_x = (center_x - x) / width * 100 
            
            # ### NUEVO: Suma de Errores (Integral) ###
            integral_x += error_x
            # ### NUEVO: Anti-Windup (Límite) ###
            integral_x = np.clip(integral_x, -LIMITE_INTEGRAL, LIMITE_INTEGRAL) 
            
            derivative_x = error_x - prev_error_x
            pid_x = (kp * error_x) + (ki * integral_x) + (kd * derivative_x)
            prev_error_x = error_x

            # === EJE Y ===
            error_y = (center_y - y) / height * 100
            
            # ### NUEVO: Suma de Errores (Integral) ###
            integral_y += error_y
            # ### NUEVO: Anti-Windup (Límite) ###
            integral_y = np.clip(integral_y, -LIMITE_INTEGRAL, LIMITE_INTEGRAL)
            
            derivative_y = error_y - prev_error_y
            pid_y = (kp * error_y) + (ki * integral_y) + (kd * derivative_y)
            prev_error_y = error_y
            
            # ### NUEVO: Mostrar valores en pantalla para calibrar ###
            texto = f"E:{error_x:.0f} SUM:{integral_x:.0f} OUT:{pid_x:.1f}"
            cv2.putText(frame, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    else:
        # ### NUEVO: Resetear suma si se pierde la pelota ###
        integral_x = 0
        integral_y = 0
        prev_error_x = 0
        prev_error_y = 0
        cv2.putText(frame, "NO DETECTADA", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    # --- COMUNICACIÓN SERIAL ---
    if ser is not None and ser.is_open:
        data = f"{pid_x:.2f},{pid_y:.2f}\n"
        try:
            ser.write(data.encode())
        except Exception as e:
            print(f"Error enviando datos: {e}")

    cv2.imshow("Camara Principal", frame)
    cv2.imshow("Mascara Filtro", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if ser:
    try: ser.write("0,0\n".encode()) 
    except: pass
    ser.close()

cap.release()
cv2.destroyAllWindows()