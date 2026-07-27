#pragma once

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "Callback.hpp"

// Teste 
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>


class MQTT {
  private:
  const char* mqtt_server = "maqiatto.com";
  const int mqtt_port = 1883;
  const char* mqtt_user = "ricardofonsecaj123@gmail.com";
  const char* mqtt_password = "12345678";
  
  WiFiClient espClient;
  PubSubClient clientMQTT;

  unsigned long lastReconnectAttempt = 0;
  bool reconectar(); 

  static MQTT* instancia;
  static void callbackRouter(char* topic, byte* payload, unsigned int length);
  
  // Teste Core 2
  TaskHandle_t mqttTaskHandle;
  static void taskMQTT(void *pvParameters);
  SemaphoreHandle_t mqttMutex;

  public:
  const char* topico_telemetria = "ricardofonsecaj123@gmail.com/telemetria";
  const char* topico_config = "ricardofonsecaj123@gmail.com/config";
  const char* topico_feedback = "ricardofonsecaj123@gmail.com/feedback"; 

  bool mqttConnected = false;
  
  void gerenciar_MQTT();
  void conectar_mqtt();
  void iniciarTaskMQTT();// teste
  void publicar_telemetria(float* dados,const char** Nome_dados,size_t quantidade_dados, const char* topico);
  };       