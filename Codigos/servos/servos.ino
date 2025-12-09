#include <ESP32Servo.h> 

Servo servoA;
Servo servoB;
Servo servoC;

// --- CONFIGURACIÓN DE PINES ---
const int PIN_A = 16; 
const int PIN_B = 17;
const int PIN_C = 18; 

// --- CONFIGURACIÓN MECÁNICA ---
const int CENTER_ANGLE = 90; // Ángulo plano
const int MAX_TILT = 35;     // Máxima inclinación (+/- grados)

// Variables de comunicación
String inputString = "";         
bool dataReady = false;  

void setup() {
  Serial.begin(115200);
  
  // Asignar timers ESP32
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoA.setPeriodHertz(50);
  servoB.setPeriodHertz(50);
  servoC.setPeriodHertz(50);

  servoA.attach(PIN_A, 500, 2400);
  servoB.attach(PIN_B, 500, 2400);
  servoC.attach(PIN_C, 500, 2400);

  // Inicializar plano
  moverServos(0, 0);
}

void loop() {
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
      float reqX = inputString.substring(0, commaIndex).toFloat();
      float reqY = inputString.substring(commaIndex + 1).toFloat();
      
      moverServos(reqX, reqY);
    }
    inputString = "";
    dataReady = false;
  }
}

// --- FUNCIÓN DE MEZCLA (MIXING) ---
void moverServos(float x, float y) {
  
  // Cinemática para servos a 120 grados
  // Ajusta los signos (+/-) dependiendo de si tus servos 
  // levantan o bajan el plato al aumentar el ángulo.
  
  float a = y;                         // Servo Frontal
  float b = -0.5 * y - 0.866 * x;      // Servo Trasero Derecho
  float c = -0.5 * y + 0.866 * x;      // Servo Trasero Izquierdo

  // Mapear a ángulos reales
  int angleA = CENTER_ANGLE + (int)a;
  int angleB = CENTER_ANGLE + (int)b;
  int angleC = CENTER_ANGLE + (int)c;

  // Limitar ángulos por seguridad
  angleA = constrain(angleA, CENTER_ANGLE - MAX_TILT, CENTER_ANGLE + MAX_TILT);
  angleB = constrain(angleB, CENTER_ANGLE - MAX_TILT, CENTER_ANGLE + MAX_TILT);
  angleC = constrain(angleC, CENTER_ANGLE - MAX_TILT, CENTER_ANGLE + MAX_TILT);

  servoA.write(angleA);
  servoB.write(angleB);
  servoC.write(angleC);
}