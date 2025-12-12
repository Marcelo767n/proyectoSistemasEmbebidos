import cv2
import numpy as np
import serial
import time
import os
from simple_pid import PID

# --- CONFIGURACIÓN CRÍTICA (AJUSTA ESTO SEGÚN LA PRUEBA ANTERIOR) ---
INVERTIR_X = False  # ¿El eje X reaccionaba al revés? Pon True.
INVERTIR_Y = False  # ¿El eje Y reaccionaba al revés? Pon True.

# --- PARAMETROS ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# PID TUNING (Empieza suave)
kp_init = 0.35
ki_init = 0.03
kd_init = 0.25

# --- INICIALIZAR PID ---
pid_x = PID(kp_init, ki_init, kd_init, setpoint=0)
pid_y = PID(kp_init, ki_init, kd_init, setpoint=0)
pid_x.output_limits = (-45, 45)
pid_y.output_limits = (-45, 45)
zona_muerta = 15.0 

def nothing(x): pass

def iniciar_camara():
    indices = [0, 1, 4, 6, 8, 2]
    print("Buscando cámara...")
    for i in indices:
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                print(f"Camara encontrada en {i}")
                return cap
            cap.release()
    return None

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    time.sleep(2)
except: ser = None

cap = iniciar_camara()
if not cap: exit()

cv2.namedWindow("Control PID")
cv2.createTrackbar("H Min", "Control PID", 90, 179, nothing)
cv2.createTrackbar("H Max", "Control PID", 130, 179, nothing)
cv2.createTrackbar("S Min", "Control PID", 120, 255, nothing)
cv2.createTrackbar("S Max", "Control PID", 255, 255, nothing)
cv2.createTrackbar("V Min", "Control PID", 100, 255, nothing)
cv2.createTrackbar("V Max", "Control PID", 255, 255, nothing)
cv2.createTrackbar("Kp", "Control PID", int(kp_init*100), 500, nothing)
cv2.createTrackbar("Ki", "Control PID", int(ki_init*1000), 100, nothing) 
cv2.createTrackbar("Kd", "Control PID", int(kd_init*100), 500, nothing)

print("SISTEMA CORRIENDO. Ajusta Kp si vibra mucho.")

while True:
    ret, frame = cap.read()
    if not ret: break
    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2

    # PID Update
    kp = cv2.getTrackbarPos("Kp", "Control PID") / 100.0
    ki = cv2.getTrackbarPos("Ki", "Control PID") / 1000.0
    kd = cv2.getTrackbarPos("Kd", "Control PID") / 100.0
    pid_x.tunings = (kp, ki, kd)
    pid_y.tunings = (kp, ki, kd)

    # Vision
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([cv2.getTrackbarPos("H Min", "Control PID"), cv2.getTrackbarPos("S Min", "Control PID"), cv2.getTrackbarPos("V Min", "Control PID")])
    upper = np.array([cv2.getTrackbarPos("H Max", "Control PID"), cv2.getTrackbarPos("S Max", "Control PID"), cv2.getTrackbarPos("V Max", "Control PID")])
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw Deadzone
    cv2.rectangle(frame, (int(cx-zona_muerta), int(cy-zona_muerta)), (int(cx+zona_muerta), int(cy+zona_muerta)), (100,100,100), 1)
    
    out_x, out_y = 0, 0

    if contours:
        c = max(contours, key=cv2.contourArea)
        ((x, y), r) = cv2.minEnclosingCircle(c)
        
        if r > 5:
            cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 255), 2)
            cv2.line(frame, (cx, cy), (int(x), int(y)), (255, 0, 0), 2)
            
            # --- ERROR & PID ---
            err_x = (x - cx) / w * 100
            err_y = (y - cy) / h * 100
            
            # Zona Muerta
            if abs(x - cx) < zona_muerta: 
                err_x = 0
                pid_x.reset()
            if abs(y - cy) < zona_muerta: 
                err_y = 0
                pid_y.reset()

            out_x = pid_x(err_x)
            out_y = pid_y(err_y)
            
            # --- INVERSIÓN FINAL ---
            if INVERTIR_X: out_x = -out_x
            if INVERTIR_Y: out_y = -out_y

            cv2.putText(frame, f"OUT: {out_x:.1f}, {out_y:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        pid_x.reset()
        pid_y.reset()

    if ser and ser.is_open:
        ser.write(f"{out_x:.2f},{out_y:.2f}\n".encode())

    cv2.imshow("Control PID", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

ser.close() if ser else None
cap.release()
cv2.destroyAllWindows()