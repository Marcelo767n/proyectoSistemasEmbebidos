#include "funciones.h"


void setup() {
  Serial.begin(115200); // El código de Python debe coincidir con este baudrate
  servoX.attach(PIN_X);
  servoY.attach(PIN_Y);
  
  // Posición inicial plana
  servoX.write(90);
  servoY.write(90);
}

void loop() {
  if (stringComplete) {
    // El formato esperado es "X,Y" (ej: "85,95")
    int commaIndex = inputString.indexOf(',');
    if (commaIndex > 0) {
      int valX = inputString.substring(0, commaIndex).toInt();
      int valY = inputString.substring(commaIndex + 1).toInt();
      
      // Seguridad mecánica: Limita los ángulos para no romper el trupan
      valX = constrain(valX, 60, 120); 
      valY = constrain(valY, 60, 120);

      servoX.write(valX);
      servoY.write(valY);
    }
    inputString = "";
    stringComplete = false;
  }
}

// Lectura eficiente del puerto serie
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
// NOTA: En ESP32, 'serialEvent' no es nativo como en UNO. 
// Mueve el contenido de serialEvent() al inicio del void loop() si no te funciona.