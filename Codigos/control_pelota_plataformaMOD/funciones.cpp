#include "funciones.h"

Servo servoX;
Servo servoY;

String inputString = "";
bool stringComplete = false;

void iniciar() {
  Serial.begin(BAUDRATE);

  servoX.attach(PIN_X);
  servoY.attach(PIN_Y);

  servoX.write(ANG_INI);
  servoY.write(ANG_INI);
}

void funcionprincipal() {

  if (stringComplete) {

    int commaIndex = inputString.indexOf(',');

    if (commaIndex > 0) {

      int valX = inputString.substring(0, commaIndex).toInt();
      int valY = inputString.substring(commaIndex + 1).toInt();

      valX = constrain(valX, ANG_MIN, ANG_MAX);
      valY = constrain(valY, ANG_MIN, ANG_MAX);

      servoX.write(valX);
      servoY.write(valY);
    }

    inputString = "";
    stringComplete = false;
  }
}

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
