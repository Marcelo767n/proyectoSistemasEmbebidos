# ⚖ Smart Balance: Sistema de Control PID con Visión Artificial

![Status](https://img.shields.io/badge/Status-En_Desarrollo-yellow)
![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![RTOS](https://img.shields.io/badge/OS-FreeRTOS-green)
![Vision](https://img.shields.io/badge/Vision-OpenCV-red)

*(Aquí inserta un GIF o Imagen de tu plataforma balanceando la pelota)*
> Un sistema embebido de tiempo real capaz de estabilizar una esfera sobre una plataforma móvil mediante retroalimentación visual.

---

## 📖 Descripción del Proyecto

Este proyecto implementa un sistema de control de lazo cerrado tipo *PID (Proporcional-Integral-Derivativo)* para controlar la posición de una pelota sobre una plataforma de 2 grados de libertad.

A diferencia de los sistemas tradicionales que usan paneles táctiles resistivos, este proyecto utiliza *Visión Artificial* externa para detectar las coordenadas $(X, Y)$ de la pelota, transmitirlas vía Serial al *ESP32, donde un sistema basado en **FreeRTOS* procesa el control y acciona los servomotores.

---

## ⚙ Arquitectura del Sistema

### 1. Diagrama de Hardware
El sistema se compone de dos bloques principales: el Procesamiento de Imagen (PC/Raspberry) y el Control en Tiempo Real (ESP32).

*(Sube una imagen a la carpeta /docs llamada 'diagrama_bloques.png' que muestre: Cámara -> PC -> (Cable USB/Serial) -> ESP32 -> Servomotores)*
![Diagrama de Bloques](./docs/diagrama_bloques.png)

* *Sensor:* Webcam / Cámara de Smartphone (Procesado con OpenCV).
* *Controlador:* ESP32 DevKit V1.
* *Actuadores:* 2x Servomotores (MG996R o similar).
* *Mecanismo:* Plataforma con articulación universal (Cardan).

### 2. Diseño de Software (FreeRTOS)
Para garantizar la estabilidad del PID, el tiempo de muestreo debe ser determinista. Se utiliza FreeRTOS para orquestar las tareas críticas.

| Tarea | Prioridad | Frecuencia | Descripción |
| :--- | :---: | :---: | :--- |
| *Task_SerialRx* | Alta | Interrupción | Recibe las coordenadas (X,Y) desde la visión artificial. Usa colas para proteger los datos. |
| *Task_PID* | Media | 20-50ms | Calcula el error de posición y la salida del algoritmo PID. |
| *Task_Servo* | Baja | On-Demand | Aplica la señal PWM a los motores basándose en el resultado del PID. |

*(Sube un diagrama de flujo o esquema de tareas a /docs)*
![Arquitectura FreeRTOS](./docs/diagrama_flujo_rtos.png)

*Justificación del uso de RTOS:*
El cálculo del PID es sensible al jitter (variación de tiempo). Si el microcontrolador se bloquea leyendo el puerto Serial, el cálculo del PID se retrasa y la pelota se cae. FreeRTOS permite que la recepción de datos interrumpa cualquier proceso menos crítico, asegurando que siempre tengamos la posición más reciente antes de calcular.

---

## 🚀 Instalación y Uso

### Requisitos
* *Hardware:* ESP32, Servos, Cámara Web.
* *Software:* Arduino IDE, Python 3.9+, OpenCV (pip install opencv-python).

### Pasos
1. *Firmware:*
   - Abrir la carpeta /firmware en Arduino IDE.
   - Instalar las librerías de FreeRTOS (si no están incluidas en el core de ESP32).
   - Cargar el código al ESP32.
2. *Visión:*
   - Conectar la cámara al PC.
   - Ejecutar el script de visión:
     bash
     cd vision_python
     python main.py
     
3. *Operación:*
   - El sistema iniciará en modo "Calibración". Coloque la pelota en el centro.

---

## 📊 Estado del Proyecto (Avance)

Según la rúbrica de evaluación, el estado actual es:

- [x] *Construcción Mecánica:* Plataforma ensamblada y servos funcionales.
- [x] *Visión Artificial:* Detección de color/forma y obtención de coordenadas X,Y.
- [x] *Comunicación Serial:* El ESP32 recibe correctamente los datos del script de Python.
- [x] *Implementación FreeRTOS:* Tareas creadas y sincronizadas.
- [ ] *Sintonización PID:* Ajuste fino de las constantes Kp, Ki, Kd (En proceso).

---

## 📸 Galería

![foto](imagene\Imagen de WhatsApp 2025-11-29 a las 01.12.45_f69b1384.jpg)
<img src="./docs/esquema_conexion.png" width="45%"> <img src="./docs/vision_screenshot.png" width="45%">

---

## 👥 Autores
* *Marcelo Navarro*
* *MIrko Ayala*
* *Roberto Ayllon*
* *Dayana Andrade