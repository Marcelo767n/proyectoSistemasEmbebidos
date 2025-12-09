#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>
#include <ESP32Servo.h>

// --- DEFINICIONES DE PINES (MACROS) ---
#define PIN_A 16
#define PIN_B 17
#define PIN_C 18

// --- CONFIGURACIÓN MECÁNICA (MACROS) ---
#define CENTER_ANGLE 90    // Ángulo plano
#define MAX_TILT 35        // Máxima inclinación (+/- grados)

// --- CONFIGURACIÓN DE SERVOS (MACROS) ---
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2400
#define SERVO_FREQ   50

// --- DECLARACIÓN DE FUNCIONES ---
// Inicializa los servos y el puerto serie
void initSystem();

// Función principal que lee serie y mueve los servos (llamar en loop)
void handleSerialLoop();

// Función interna de cinemática (accesible si se necesita externamente)
void moverServos(float x, float y);

#endif