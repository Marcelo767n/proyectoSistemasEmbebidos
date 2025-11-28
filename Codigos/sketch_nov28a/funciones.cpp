/*
 * ARCHIVO DE IMPLEMENTACIÓN (.cpp)
 */

#include <WiFi.h>      // Librería exclusiva del ESP32
#include "funciones.h" 

// --- TUS DATOS DE WIFI (CÁMBIALOS) ---
const char* ssid = "TU_SSID";          
const char* password = "TU_PASSWORD";  

// Objetos globales
WiFiServer server(8080); 
WiFiClient client;       

void iniciarWifi() {
    Serial.println();
    Serial.print("Conectando a: ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    // Esperar conexión
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("¡WiFi Conectado!");
    Serial.print("IP del ESP32: ");
    Serial.println(WiFi.localIP()); 

    server.begin();
    Serial.println("Servidor TCP iniciado en puerto 8080");
}

void comunicarConPython() {
    // 1. SI NO HAY CLIENTE, INTENTAR ACEPTAR UNO NUEVO
    if (!client || !client.connected()) {
        client = server.accept(); 
        
        if (client) {
            Serial.println("-> Nuevo Cliente (Python) conectado");
            client.flush(); 
        }
        return; 
    }

    // 2. SI HAY CLIENTE, VERIFICAR SI ENVIÓ DATOS
    if (client.available()) {
        String mensaje = client.readStringUntil('\n');
        mensaje.trim(); 

        if (mensaje.length() > 0) {
            Serial.print("Comando recibido: ");
            Serial.println(mensaje);

            // Responder a Python
            client.println("RECIBIDO: " + mensaje);
            
            // Ejemplo de lógica futura:
            if (mensaje == "START") {
                Serial.println("Iniciando...");
            }
        }
    }
}