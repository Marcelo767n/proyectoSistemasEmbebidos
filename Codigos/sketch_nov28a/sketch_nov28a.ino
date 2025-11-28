#include "funciones.h"

void setup() {
    Serial.begin(115200);
    iniciarWifi();
}

void loop() {
    comunicarConPython();
    delay(10);
}