#pragma once
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


class Gerenciador_MQTT {
  private:
  const char* mqtt_server = "maqiatto.com";
  const int mqtt_port = 1883;
  const char* mqtt_user = "ricardofonsecaj123@gmail.com";
  const char* mqtt_password = "12345678";
  const char* topico_telemetria = "ricardofonsecaj123@gmail.com/telemetria";
  const char* topico_config = "ricardofonsecaj123@gmail.com/config";
  const char* topico_feedback = "ricardofonsecaj123@gmail.com/feedback"; 

  WiFiClient espClient;
  PubSubClient clientMQTT;

  unsigned long lastReconnectAttempt = 0;
  bool reconectar(); 

  public:
    bool mqttConnected = false;
    bool parametros_recebidos = false;
    int qtd_recebida = 0;
    
    char nomes_recebidos[10][20];  // Guarda os nomes ("kp", "rpm", etc)
    float valores_recebidos[10];   // Guarda os números (1.5, 1200.0, etc)

    void Gerenciar_MQTT();
    void conectar_mqtt();
    void publicar_telemetria(float* dados,const char** Nome_dados,size_t quantidade_dados, const char* topico);
    void publicar_feedback(const char* topico, const char* mensagem_feedback, float* dados,const char** Nome_dados,size_t quantidade_dados);
    void extrair_parametros(byte* payload, unsigned int length);
};       
