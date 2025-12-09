#ifndef FUNCIONES_H
#define FUNCIONES_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ===== CONFIGURACIÓN =====
#define PIN_X 18
#define PIN_Y 19
#define PIN_Z 21

#define BAUDRATE 115200
#define DELAY 1000
#define T_DELAY 10
#define ANG_MIN 0
#define ANG_MAX 180
#define ANG_START 90

// Estructura para enviar ángulos
typedef struct {
  int x, y, z;
} ServoAngles;

extern QueueHandle_t colaServos;

// Funciones públicas
void iniciarSistema();
void tareaSerial(void *pv);
void tareaServos(void *pv);

#endif
