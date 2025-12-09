#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// --- 1. CONFIGURACIÓN DE COMUNICACIÓN ---
#define VELOCIDAD_SERIAL      115200
#define TIEMPO_ESPERA_COLA    10      // Ticks a esperar si la cola está llena

// --- 2. CONFIGURACIÓN DE PINES ---
#define PIN_SERVO_A           18
#define PIN_SERVO_B           19
#define PIN_SERVO_C           21

// --- 3. CONFIGURACIÓN MECÁNICA ---
#define ANGULO_CENTRO         90      // Ángulo plano
#define INCLINACION_MAX       35      // Máxima inclinación (+/- grados)

// --- 4. CONFIGURACIÓN DE SERVOS ---
#define SERVO_MIN_US          500
#define SERVO_MAX_US          2400
#define SERVO_FREQ_HZ         50

// --- 5. CONFIGURACIÓN FREERTOS ---
// Tamaños de pila (Stack)
#define STACK_TAREA_SERIAL    2048
#define STACK_TAREA_SERVO     2048

// Prioridades (Mayor número = Mayor prioridad)
#define PRIORIDAD_SERIAL      1       // Prioridad baja (IO)
#define PRIORIDAD_SERVO       2       // Prioridad alta (Movimiento real)

// Tiempos y Delays
#define DELAY_LOOP_SERIAL_MS  10      // Delay para no saturar la tarea serial (ms)
#define LONGITUD_COLA         5       // Capacidad del buffer de comandos

// --- ESTRUCTURA DE DATOS ---
struct Coordenadas {
    float x;
    float y;
};

// --- DECLARACIÓN DE FUNCIONES ---
void inicializarSistema();
void moverServos(float x, float y); // Cinemática interna

// Tareas de FreeRTOS
void TareaSerial(void *pvParameters);
void TareaServo(void *pvParameters);

#endif