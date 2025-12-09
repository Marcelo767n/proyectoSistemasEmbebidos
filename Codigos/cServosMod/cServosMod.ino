#include "funciones.h"

void setup() {
  // Toda la configuración está encapsulada aquí
  initSystem();
}

void loop() {
  // El manejo de serie y servos se realiza aquí
  handleSerialLoop();
}