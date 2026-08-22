#pragma once
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class Gerenciador_MQTT {
  private:
    const char* mqtt_server = "maqiatto.com";
    const int mqtt_port = 1883;
    const char* mqtt_user = "ricardofonsecaj123@gmail.com";
    const char* mqtt_password = "12345678";
  
    WiFiClient espClient;
    PubSubClient clientMQTT;

    unsigned long lastReconnectAttempt = 0;
    bool reconectar(); 

    static Gerenciador_MQTT* instancia;
    static void callbackRouter(char* topic, byte* payload, unsigned int length);
    
    // Threads
    TaskHandle_t mqttTaskHandle;
    static void taskMQTT(void *pvParameters);
    SemaphoreHandle_t mqttMutex;

  public:
    const char* topico_telemetria = "ricardofonsecaj123@gmail.com/telemetria";
    const char* topico_config = "ricardofonsecaj123@gmail.com/config";
    const char* topico_feedback = "ricardofonsecaj123@gmail.com/feedback"; 

    bool mqttConnected = false;
    
    void Gerenciar_MQTT();
    void conectar_mqtt();
    void iniciarTaskMQTT();
    
    // A NOVA FUNÇÃO GERAL DE ENVIO
    void publicar_telemetria(const JsonDocument& doc, const char* topico);
};