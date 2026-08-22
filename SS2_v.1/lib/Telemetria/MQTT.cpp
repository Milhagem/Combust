#include "MQTT.hpp"
#include "Callback.hpp" 

Gerenciador_MQTT* Gerenciador_MQTT::instancia = nullptr;

void Gerenciador_MQTT::conectar_mqtt() {
    instancia = this; 

    if (mqttMutex == NULL) {
        mqttMutex = xSemaphoreCreateMutex(); 
    }

    clientMQTT.setClient(espClient);
    clientMQTT.setServer(mqtt_server, mqtt_port);
    clientMQTT.setCallback(callbackRouter); 
    
    clientMQTT.setSocketTimeout(1); 
    clientMQTT.setBufferSize(1024); // Garantindo buffer grande para JSONs unificados
}

void Gerenciador_MQTT::callbackRouter(char* topic, byte* payload, unsigned int length) {
    if (instancia != nullptr) {
        if (strcmp(topic, instancia->topico_config) == 0) {
            Callback::processarMensagem(payload, length, instancia->clientMQTT, instancia->topico_feedback);
        }
    }
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
    if (mqttMutex != NULL && xSemaphoreTake(mqttMutex, (TickType_t)10) == pdTRUE) {
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
        xSemaphoreGive(mqttMutex);
    }
}

// ==========================================
// FUNÇÃO GERAL E ÚNICA DE PUBLICAÇÃO
// ==========================================
void Gerenciador_MQTT::publicar_telemetria(const JsonDocument& doc, const char* topico) {
    if (mqttMutex != NULL && xSemaphoreTake(mqttMutex, (TickType_t)20) == pdTRUE) {
        if (clientMQTT.connected()) {
            char payloadMQTT[1024];
            serializeJson(doc, payloadMQTT);
            clientMQTT.publish(topico, payloadMQTT);
        }
        xSemaphoreGive(mqttMutex);
    }
}

// ==========================================
// THREADS FREERTOS
// ==========================================
void Gerenciador_MQTT::iniciarTaskMQTT() {
    xTaskCreatePinnedToCore(
        taskMQTT, "Task_MQTT", 4096, this, 1, &mqttTaskHandle, 0
    );
}

void Gerenciador_MQTT::taskMQTT(void *pvParameters) {
    Gerenciador_MQTT* gerenciador = (Gerenciador_MQTT*)pvParameters;
    for (;;) { 
        gerenciador->Gerenciar_MQTT();
        vTaskDelay(20 / portTICK_PERIOD_MS); 
    }
}