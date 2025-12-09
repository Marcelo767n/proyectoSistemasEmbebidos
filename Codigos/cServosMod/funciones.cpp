#include "funciones.h"

// --- OBJETOS GLOBALES (Solo visibles en este archivo) ---
Servo servoA;
Servo servoB;
Servo servoC;

// Variables de comunicación
String inputString = "";
bool dataReady = false;

// --- IMPLEMENTACIÓN DE FUNCIONES ---

void initSystem() {
  Serial.begin(115200);

  // Asignar timers ESP32 (Necesario para la librería ESP32Servo)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoA.setPeriodHertz(SERVO_FREQ);
  servoB.setPeriodHertz(SERVO_FREQ);
  servoC.setPeriodHertz(SERVO_FREQ);

  servoA.attach(PIN_A, SERVO_MIN_US, SERVO_MAX_US);
  servoB.attach(PIN_B, SERVO_MIN_US, SERVO_MAX_US);
  servoC.attach(PIN_C, SERVO_MIN_US, SERVO_MAX_US);

  // Inicializar en posición plana
  moverServos(0, 0);
  Serial.println("Sistema Iniciado. Esperando comandos (x,y)...");
}

void moverServos(float x, float y) {
  // Cinemática para servos a 120 grados
  // Ecuaciones:
  // A = y
  // B = -0.5y - (sqrt(3)/2)x  -> 0.866 es aprox sqrt(3)/2
  // C = -0.5y + (sqrt(3)/2)x

  float a = y;
  float b = -0.5 * y - 0.866 * x;
  float c = -0.5 * y + 0.866 * x;

  // Mapear a ángulos reales
  int angleA = CENTER_ANGLE + (int)a;
  int angleB = CENTER_ANGLE + (int)b;
  int angleC = CENTER_ANGLE + (int)c;

  // Limitar ángulos por seguridad (Constrain)
  angleA = constrain(angleA, CENTER_ANGLE - MAX_TILT, CENTER_ANGLE + MAX_TILT);
  angleB = constrain(angleB, CENTER_ANGLE - MAX_TILT, CENTER_ANGLE + MAX_TILT);
  angleC = constrain(angleC, CENTER_ANGLE - MAX_TILT, CENTER_ANGLE + MAX_TILT);

  servoA.write(angleA);
  servoB.write(angleB);
  servoC.write(angleC);
}

void handleSerialLoop() {
  // 1. Leer datos del puerto serie
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      dataReady = true;
    } else {
      inputString += inChar;
    }
  }

  // 2. Procesar datos si llegó un paquete completo
  if (dataReady) {
    int commaIndex = inputString.indexOf(',');

    if (commaIndex > 0) {
      // Convertir substrings a float
      float reqX = inputString.substring(0, commaIndex).toFloat();
      float reqY = inputString.substring(commaIndex + 1).toFloat();

      moverServos(reqX, reqY);
    }
    // Limpiar variables
    inputString = "";
    dataReady = false;
  }
}