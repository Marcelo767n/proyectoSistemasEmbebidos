import cv2 as cv
import numpy as np
import serial
import time

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