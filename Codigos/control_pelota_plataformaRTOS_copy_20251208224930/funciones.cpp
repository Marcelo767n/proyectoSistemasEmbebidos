#include "funciones.h"

Servo servoX, servoY, servoZ;
QueueHandle_t colaServos;

// ------------ TAREA SERIAL ------------
void tareaSerial(void *pv) {
  String buffer = "";

  for (;;) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == '\n') {
        int p1 = buffer.indexOf(',');
        int p2 = buffer.lastIndexOf(',');

        if (p1 > 0 && p2 > p1) {
          ServoAngles pos;
          pos.x = constrain(buffer.substring(0, p1).toInt(), ANG_MIN, ANG_MAX);
          pos.y = constrain(buffer.substring(p1 + 1, p2).toInt(), ANG_MIN, ANG_MAX);
          pos.z = constrain(buffer.substring(p2 + 1).toInt(), ANG_MIN, ANG_MAX);

          xQueueSend(colaServos, &pos, 0);
        }
        buffer = "";
      } else {
        buffer += c;
      }
    }
    vTaskDelay(T_DELAY);
  }
}

// ------------ TAREA SERVOS ------------
void tareaServos(void *pv) {
  ServoAngles pos = {ANG_START, ANG_START, ANG_START};

  for (;;) {
    if (xQueueReceive(colaServos, &pos, portMAX_DELAY)) {

      servoX.write(pos.x);
      servoY.write(pos.y);
      servoZ.write(pos.z);

      Serial.printf("Servos -> X:%d  Y:%d  Z:%d\n", pos.x, pos.y, pos.z);
    }
  }
}

// ------------ INICIO DEL SISTEMA ------------
void iniciarSistema() {
  Serial.begin(BAUDRATE);

  servoX.attach(PIN_X);
  servoY.attach(PIN_Y);
  servoZ.attach(PIN_Z);

  servoX.write(ANG_START);
  servoY.write(ANG_START);
  servoZ.write(ANG_START);

  colaServos = xQueueCreate(5, sizeof(ServoAngles));

  xTaskCreatePinnedToCore(tareaSerial, "Serial", 2000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(tareaServos, "Servos", 2000, NULL, 2, NULL, 1);
}
