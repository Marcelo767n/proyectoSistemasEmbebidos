# 🧠 Smart Balance  
### Proyecto Final — Sistemas Embebidos I  
Universidad Católica Boliviana — Ingeniería Mecatrónica

---

## 📌 Descripción General

**Smart Balance** es un sistema embebido diseñado para estabilizar una pelota sobre una plataforma mediante control inteligente y retroalimentación en tiempo real.

El proyecto combina:

- Control con microcontroladores  
- Comunicación inalámbrica WiFi  
- Procesamiento en Python  
- Control modular en Arduino IDE  
- Sensado, actuadores y electrónica embebida  
- Integración hardware–software

Este repositorio contiene **todo el código, documentación y recursos** necesarios para ejecutar el sistema.

---

## 🟦 Características del Proyecto

- ✔ **Control en tiempo real** de la posición de la pelota  
- ✔ **Código en Arduino completamente modularizado**  
- ✔ **Comunicación por WiFi con la computadora**  
- ✔ **Interfaz en Python para el procesamiento y envío de datos**  
- ✔ **Control dual por ejes X–Y**  
- ✔ Diseño accesible para laboratorios educativos  
- ✔ Código claro, documentado y fácil de extender  

---

## 📡 Arquitectura del Sistema

- 🖥 **Computadora (Python)**  
  - Procesamiento de la posición  
  - Envío de información al microcontrolador  
  - Comunicación **WiFi**

- 📶 **Módulo WiFi / Microcontrolador**  
  - Recepción de datos desde la computadora  
  - Cálculo de control  
  - Comunicación UART con Arduino

- 🔧 **Arduino (IDE)**  
  - Código modular  
  - Control de servomotores  
  - Ejecución del controlador y actuadores

---

## 🔧 Estructura del Repositorio