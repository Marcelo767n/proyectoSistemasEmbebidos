#include "funciones.h"

void inciar (){
  Serial.begin(BAUDRATE); // El código de Python debe coincidir con este baudrate
  servoX.attach(PIN_X);
  servoY.attach(PIN_Y);
  
  // Posición inicial plana
  servoX.write(ANG_INI);
  servoY.write(ANG_INI);
}