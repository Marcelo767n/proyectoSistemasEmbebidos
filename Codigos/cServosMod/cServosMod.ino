#include "funciones.h"

void setup() {
  inicializarSistema();
}

void loop() {
  // Eliminamos la tarea del loop por limpieza, ya que usamos tareas dedicadas
  vTaskDelete(NULL); 
}