#ifndef FUNCIONES_H
#define FUNCIONES_H
#include <Arduino.h>
Servo servoX;
Servo servoY;

// Pines PWM del ESP32 
#define PIN_X  18
#define PIN_Y  19
#define BAUDRATE 115200
#define ANG_INI 90
#define ANG_MIN 60
#define ANG_MAX 120
String inputString = "";         
boolean stringComplete = false;  

void iniciar();
#endif