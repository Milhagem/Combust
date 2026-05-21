#include "Telemetria.hpp"

Telemetria::Telemetria()
    : ssid("Diogo's Galaxy M62"),
      password("awur7323"),
      mqtt_server("broker.hivemq.com"),
      espClient(),
      client(espClient),
      lastMsg(0)
{
}

// ================= CONEXÃO WI-FI ===================
void Telemetria::setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Conectando a ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi conectado!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());
    client.setServer(mqtt_server, 1883);
}

// ======== RECONEXÃO MQTT ========
void Telemetria::reconnect() {
    if (!client.connected()) {
        unsigned long now = millis();
        if (now - lastMsg > 5000) {
            lastMsg = now;
            Serial.print("Tentando conexão MQTT...");
            String clientId = "Milhagem-Device-" + String(random(0xffff), HEX);
            if (client.connect(clientId.c_str())) {
                Serial.println("Conectado ao broker!");
            } else {
                Serial.print("falhou, rc=");
                Serial.print(client.state());
                Serial.println(" - tentando novamente no próximo ciclo");
            }
        }
    }
}

// ======== ENVIO WIFI ========
void Telemetria::EnviodadosWifi(float accel, float pitch, float yaw, float lat, float lon) {
    reconnect();
    if (client.connected()) {
        client.loop();
        snprintf(payload, sizeof(payload),
            "{\"Acelera\":%.2f,\"Inclinacao\":%.2f,\"Rotacao\":%.2f,\"Lat\":%.6f,\"Long\":%.6f}",
            accel, pitch, yaw, lat, lon);

        Serial.print("Publicando: ");
        Serial.println(payload);
        client.publish("sensor/esp32S3", payload);
    }
}