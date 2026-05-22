#include "Telemetria.hpp"

Telemetria::Telemetria()
    : ssid("Diogo's Galaxy M62"),
      password("awur7323"),
      mqtt_server("broker.hivemq.com"),
      espClient(),
      client(espClient),
      lastMsg(0),
      wifi_initialized(false)
{
}

// ================= CONEXÃO WI-FI ===================
void Telemetria::init_wifi() {
    if (wifi_initialized) return;  // Já foi inicializado
    
    Serial.println("[WiFi] Conectando...");
    WiFi.begin(ssid, password);
    
    // Aguarda até 20 segundos
    for (int attempts = 0; attempts < 40; attempts++) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[WiFi] Conectado!");
            client.setServer(mqtt_server, 1883);
            wifi_initialized = true;
            return;
        }
        delay(500);
    }
    
    Serial.println("[WiFi] Falha na conexão");
}

// ======== RECONEXÃO MQTT ========
void Telemetria::reconnect() {
    if (!wifi_initialized) return;  // WiFi não está pronto
    
    if (!client.connected()) {
        unsigned long now = millis();
        if (now - lastMsg > 5000) {
            lastMsg = now;
            String clientId = "Milhagem-Device-" + String(random(0xffff), HEX);
            if (client.connect(clientId.c_str())) {
                Serial.println("[MQTT] Conectado");
            } else {
                // Evitar spam de logs
                // Serial.print("[MQTT] Falha: ");
                // Serial.println(client.state());
            }
        }
    }
}

// ======== ENVIO WIFI ========
void Telemetria::EnviodadosWifi(float accel, float pitch, float yaw, float lat, float lon) {
    if (!wifi_initialized) return;
    
    reconnect();
    if (client.connected()) {
        client.loop();
        snprintf(payload, sizeof(payload),
            "{\"Acelera\":%.2f,\"Inclinacao\":%.2f,\"Rotacao\":%.2f,\"Lat\":%.6f,\"Long\":%.6f}",
            accel, pitch, yaw, lat, lon);

        // Debug em ciclos espaçados
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 10000) {  // A cada 10s
            lastDebug = millis();
            Serial.print("[MQTT] ");
            Serial.println(payload);
        }
        
        client.publish("sensor/esp32S3", payload);
    }
}