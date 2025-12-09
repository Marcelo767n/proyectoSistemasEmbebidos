#include "funciones.h"

void setup() {
  iniciarSistema();
}

void loop() {
  // FreeRTOS maneja las tareas → loop queda vacío
  delay(DELAY);
}
