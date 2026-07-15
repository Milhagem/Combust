#include "Gerenciador_MQTT.hpp"

void Gerenciador_MQTT::conectar_mqtt() {

    clientMQTT.setClient(espClient);
    clientMQTT.setServer(mqtt_server, mqtt_port);

    clientMQTT.setCallback([this](char* topic, byte* payload, unsigned int length) {
        this->extrair_parametros(payload, length);
        
                Serial.println("Parametros atualizados via MQTT!");
    });
}


bool Gerenciador_MQTT::reconectar() {
    if (clientMQTT.connect("CarroMilhagem", mqtt_user, mqtt_password)) {
   
        mqttConnected = true;
        clientMQTT.subscribe(topico_config); 
        return true;
    }
    return false;
}

void Gerenciador_MQTT::Gerenciar_MQTT() {
     if (!clientMQTT.connected()) {
        mqttConnected = false;
        unsigned long now = millis();
        if (now - lastReconnectAttempt > 5000) { 
            lastReconnectAttempt = now; 
            if (reconectar()) 
            lastReconnectAttempt = 0; 
        }
    } else { 
        clientMQTT.loop(); 
    }
}


void Gerenciador_MQTT::publicar_telemetria(float* dados, const char** Nome_dados, size_t quantidade_dados,const char* topico) {
    if (!clientMQTT.connected()) {
        return;
    }

    JsonDocument doc; 

    for (size_t i = 0; i < quantidade_dados; ++i) {
        doc[Nome_dados[i]] = dados[i];
    }

    char payloadMQTT[512];

    serializeJson(doc, payloadMQTT);

    clientMQTT.publish(topico, payloadMQTT);
}

void Gerenciador_MQTT::publicar_feedback(const char* topico, const char* mensagem_feedback, float* dados, const char** Nome_dados, size_t quantidade_dados) {
    if (!clientMQTT.connected()) {
        return;
    }

    JsonDocument doc; 
    doc["mensagem"] = mensagem_feedback;

    for (size_t i = 0; i < quantidade_dados; ++i) {
        doc[Nome_dados[i]] = dados[i];
    }

    char payloadMQTT[512];

    serializeJson(doc, payloadMQTT);

    clientMQTT.publish(topico, payloadMQTT);
}

void Gerenciador_MQTT::extrair_parametros(byte* payload, unsigned int length) {
    JsonDocument doc; 
    
    if (deserializeJson(doc, payload, length)) {
        publicar_feedback(topico_feedback, "ERRO: Arquivo JSON recebido e invalido ou corrompido.", nullptr, nullptr, 0);
        Serial.println("ERRO: Arquivo JSON recebido e invalido ou corrompido.");
        return; 
    }

    qtd_recebida = 0; 
    JsonObject objeto = doc.as<JsonObject>();

    for (JsonPair dado : objeto) {
        if (qtd_recebida >= 10) break; 

        strlcpy(nomes_recebidos[qtd_recebida], dado.key().c_str(), sizeof(nomes_recebidos[0]));
        valores_recebidos[qtd_recebida] = dado.value().as<float>();

        qtd_recebida++; 
    }

    if (qtd_recebida > 0) {
        parametros_recebidos = true;
    }
}






