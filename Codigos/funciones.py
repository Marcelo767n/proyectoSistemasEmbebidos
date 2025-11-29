# funciones.py
import cv2
import numpy as np
import serial
import time

# ---------------------- SERIAL -----------------------------

def iniciar_serial(puerto='COM5', baudrate=115200):
    try:
        ser = serial.Serial(puerto, baudrate, timeout=0.1)
        time.sleep(2)
        print(f"[OK] Conectado a {puerto}")
        return ser
    except:
        print(f"[ADVERTENCIA] No se pudo conectar a {puerto}. Modo solo visión.")
        return None


# ---------------------- TRACKBARS ---------------------------

def nothing(x):
    pass

def crear_trackbars(kp, ki, kd):
    cv2.namedWindow("Calibracion")

    # HSV
    cv2.createTrackbar("H Min", "Calibracion", 0, 179, nothing)
    cv2.createTrackbar("H Max", "Calibracion", 25, 179, nothing)
    cv2.createTrackbar("S Min", "Calibracion", 130, 255, nothing)
    cv2.createTrackbar("S Max", "Calibracion", 255, 255, nothing)
    cv2.createTrackbar("V Min", "Calibracion", 150, 255, nothing)
    cv2.createTrackbar("V Max", "Calibracion", 255, 255, nothing)

    # PID
    cv2.createTrackbar("Kp", "Calibracion", int(kp * 100), 200, nothing)
    cv2.createTrackbar("Ki", "Calibracion", int(ki * 100), 100, nothing)
    cv2.createTrackbar("Kd", "Calibracion", int(kd * 100), 200, nothing)


def leer_trackbars():
    h_min = cv2.getTrackbarPos("H Min", "Calibracion")
    h_max = cv2.getTrackbarPos("H Max", "Calibracion")
    s_min = cv2.getTrackbarPos("S Min", "Calibracion")
    s_max = cv2.getTrackbarPos("S Max", "Calibracion")
    v_min = cv2.getTrackbarPos("V Min", "Calibracion")
    v_max = cv2.getTrackbarPos("V Max", "Calibracion")

    kp = cv2.getTrackbarPos("Kp", "Calibracion") / 100.0
    ki = cv2.getTrackbarPos("Ki", "Calibracion") / 100.0
    kd = cv2.getTrackbarPos("Kd", "Calibracion") / 100.0

    return (h_min, h_max, s_min, s_max, v_min, v_max, kp, ki, kd)


# ---------------------- SERVO MAP ---------------------------

def map_servo(value):
    angle = 90 + int(value)
    if angle > 120: angle = 120
    if angle < 60: angle = 60
    return angle


# ---------------------- DETECCIÓN ---------------------------

def detectar_pelota(frame, hsv, rangos):
    h_min, h_max, s_min, s_max, v_min, v_max = rangos

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return mask, contours


# ---------------------- PID ---------------------------

def pid_control(x, y, cx, cy, w, h, kp, ki, kd, estado):
    prev_x, prev_y, int_x, int_y = estado

    error_x = (x - cx) / w * 100
    int_x += error_x
    der_x = error_x - prev_x
    out_x = kp * error_x + ki * int_x + kd * der_x

    error_y = (y - cy) / h * 100
    int_y += error_y
    der_y = error_y - prev_y
    out_y = kp * error_y + ki * int_y + kd * der_y

    return out_x, out_y, (error_x, error_y, int_x, int_y)


# ---------------------- ENVÍO DE DATOS ---------------------------

def enviar_serial(ser, x, y):
    if ser is not None and ser.is_open:
        paquete = f"{x},{y}\n"
        ser.write(paquete.encode())
