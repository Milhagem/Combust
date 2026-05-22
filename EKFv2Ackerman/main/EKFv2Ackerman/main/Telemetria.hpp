#ifndef TELEMETRIA_HPP
#define TELEMETRIA_HPP

#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class Telemetria {
public:

    Telemetria();

    // Inicializa WiFi e MQTT (chamado uma vez em setup())
    void init_wifi();

    // Envia dados via WiFi/MQTT
    void EnviodadosWifi(float accel, float pitch, float yaw, float lat, float lon);

private:

    // ── WiFi / MQTT ───────────────────────────────────────
    char payload[512];
    const char* ssid;
    const char* password;
    const char* mqtt_server;

    WiFiClient   espClient;
    PubSubClient client;
    unsigned long lastMsg;
    bool wifi_initialized;

    void reconnect();

};

#endif 