#include "funciones.h"

// --- OBJETOS GLOBALES (Internos) ---
Servo servoA;
Servo servoB;
Servo servoC;

// Manejador de la cola
QueueHandle_t colaServos;

void inicializarSistema() {
  Serial.begin(VELOCIDAD_SERIAL);

  // 1. Configurar Timers y Servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoA.setPeriodHertz(SERVO_FREQ_HZ);
  servoB.setPeriodHertz(SERVO_FREQ_HZ);
  servoC.setPeriodHertz(SERVO_FREQ_HZ);

  servoA.attach(PIN_SERVO_A, SERVO_MIN_US, SERVO_MAX_US);
  servoB.attach(PIN_SERVO_B, SERVO_MIN_US, SERVO_MAX_US);
  servoC.attach(PIN_SERVO_C, SERVO_MIN_US, SERVO_MAX_US);

  // Posición inicial
  moverServos(0, 0);

  // 2. Crear la Cola
  colaServos = xQueueCreate(LONGITUD_COLA, sizeof(Coordenadas));

  if (colaServos == NULL) {
    Serial.println("Error crítico: No se pudo crear la cola FreeRTOS");
    while(1);
  }

  // 3. Crear Tareas FreeRTOS
  // Core 0 para comunicaciones, Core 1 para movimiento
  
  xTaskCreatePinnedToCore(
    TareaSerial, "TareaSerial", STACK_TAREA_SERIAL, NULL, PRIORIDAD_SERIAL, NULL, 0
  );

  xTaskCreatePinnedToCore(
    TareaServo, "TareaServo", STACK_TAREA_SERVO, NULL, PRIORIDAD_SERVO, NULL, 1
  );

  Serial.println("Sistema FreeRTOS Iniciado Correctamente.");
}

/// --- TAREA 1: LECTURA SERIAL OPTIMIZADA (High Speed) ---
void TareaSerial(void *pvParameters) {
  String inputString = "";
  inputString.reserve(30); // Reservar memoria para evitar fragmentación

  for (;;) {
    // 1. MIENTRAS haya datos, leemos todo lo posible sin pausas
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      
      if (inChar == '\n') {
        // Fin de paquete: Procesar inmediatamente
        int commaIndex = inputString.indexOf(',');
        
        if (commaIndex > 0) {
          Coordenadas nuevosDatos;
          // toFloat() es un poco lento, pero aceptable. 
          // Si necesitas más velocidad, se puede optimizar más con atof().
          nuevosDatos.x = inputString.substring(0, commaIndex).toFloat();
          nuevosDatos.y = inputString.substring(commaIndex + 1).toFloat();

          // Enviar a la cola SIN esperar (0 ticks) si está llena, 
          // para no bloquear la lectura de datos nuevos.
          xQueueSend(colaServos, &nuevosDatos, 0);
        }
        inputString = ""; // Limpiar buffer
      } else {
        inputString += inChar;
      }
    }
    
    // 2. Solo cuando el buffer serial está VACÍO, hacemos un pequeño delay
    // para ceder la CPU a otras tareas. 1ms es suficiente.
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}
// --- TAREA 2: MOVER SERVOS ---
void TareaServo(void *pvParameters) {
  Coordenadas datosRecibidos;

  for (;;) {
    // Espera infinita (portMAX_DELAY) hasta que llegue un dato
    if (xQueueReceive(colaServos, &datosRecibidos, portMAX_DELAY) == pdPASS) {
      moverServos(datosRecibidos.x, datosRecibidos.y);
    }
  }
}

// --- LÓGICA CINEMÁTICA ---
void moverServos(float x, float y) {
  float a = y;
  float b = -0.5 * y - 0.866 * x;
  float c = -0.5 * y + 0.866 * x;

  int angleA = ANGULO_CENTRO + (int)a;
  int angleB = ANGULO_CENTRO + (int)b;
  int angleC = ANGULO_CENTRO + (int)c;

  // Restricción usando macros
  angleA = constrain(angleA, ANGULO_CENTRO - INCLINACION_MAX, ANGULO_CENTRO + INCLINACION_MAX);
  angleB = constrain(angleB, ANGULO_CENTRO - INCLINACION_MAX, ANGULO_CENTRO + INCLINACION_MAX);
  angleC = constrain(angleC, ANGULO_CENTRO - INCLINACION_MAX, ANGULO_CENTRO + INCLINACION_MAX);

  servoA.write(angleA);
  servoB.write(angleB);
  servoC.write(angleC);
}