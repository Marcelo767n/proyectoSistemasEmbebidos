# principal.py
import cv2
from funciones import *

# ----- Parámetros iniciales PID -----
kp = 0.4
ki = 0.0
kd = 0.2

estado_pid = (0, 0, 0, 0)  # prev_x, prev_y, int_x, int_y
    
# ----- Inicialización -----
ser = iniciar_serial('COM5', 115200)
crear_trackbars(kp, ki, kd)

cap = cv2.VideoCapture(0)
print("Presiona 'q' para salir.")

# ----- LOOP PRINCIPAL -----
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2

    # --- Leer valores de Trackbars ---
    h_min, h_max, s_min, s_max, v_min, v_max, kp, ki, kd = leer_trackbars()

    # --- Detectar pelota ---
    rangos_hsv = (h_min, h_max, s_min, s_max, v_min, v_max)
    mask, contours = detectar_pelota(frame, hsv, rangos_hsv)

    servo_x = 90
    servo_y = 90

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.line(frame, (cx, cy), (int(x), int(y)), (0, 0, 255), 2)

            # --- PID ---
            out_x, out_y, estado_pid = pid_control(
                x, y, cx, cy, w, h, kp, ki, kd, estado_pid
            )

            # --- Mapeo a servo ---
            servo_x = map_servo(-out_x)
            servo_y = map_servo(out_y)

    # Enviar datos
    enviar_serial(ser, servo_x, servo_y)

    # Dibujar punto central
    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

    cv2.imshow("Camara", frame)
    cv2.imshow("Mascara", mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

if ser:
    ser.close()
