#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>
#include <ESP32Servo.h>

// Pines PWM del ESP32 
#define PIN_X  18
#define PIN_Y  19

// Serial
#define BAUDRATE 115200

// Rangos de ángulos
#define ANG_INI 90
#define ANG_MIN 60
#define ANG_MAX 120

// Variables globales (solo declaración)
extern String inputString;
extern bool stringComplete;

// Declaración de servos
extern Servo servoX;
extern Servo servoY;

// Prototipos
void iniciar();
void funcionprincipal();
void serialEvent();

#endif
