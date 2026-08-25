#include "MQTT.hpp"
#include "Callback.hpp" // Incluímos a classe que vai fazer o trabalho pesado

Gerenciador_MQTT* Gerenciador_MQTT::instancia = nullptr;

void Gerenciador_MQTT::conectar_mqtt() {
    instancia = this; 

    // Cria o Mutex usando a variável oficial da classe
    if (mqttMutex == NULL) {
        mqttMutex = xSemaphoreCreateMutex(); 
    }

    clientMQTT.setClient(espClient);
    clientMQTT.setServer(mqtt_server, mqtt_port);
    clientMQTT.setCallback(callbackRouter); 
    
   
    clientMQTT.setSocketTimeout(1); 
    clientMQTT.setBufferSize(1024);
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
        clientMQTT.subscribe(topico_config); // Assina o tópico
        return true;
    }
    return false;
}

void Gerenciador_MQTT::Gerenciar_MQTT() {
    // esse if [é um teste]
    // Tenta pegar a chave (espera no máximo 10 ticks). Se conseguir, entra:
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
        
        // Devolve a chave para o outro núcleo poder usar!
        xSemaphoreGive(mqttMutex);
    }
}

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


// testes


void Gerenciador_MQTT::iniciarTaskMQTT() {
    // Cria uma thread paralela rodando no Core 0
    xTaskCreatePinnedToCore(
        taskMQTT,           // Função que será executada na thread
        "Task_MQTT",        // Nome da task para debug
        4096,               // Tamanho da pilha de memória (Stack) em bytes
        this,               // Passa o próprio objeto (instância atual) como parâmetro
        1,                  // Prioridade da Task (1 é baixa, ideal para não atrapalhar o WiFi nativo)
        &mqttTaskHandle,    // Handle da task
        0                   // CORE 0 (A Mágica acontece aqui!)
    );
}

void Gerenciador_MQTT::taskMQTT(void *pvParameters) {
    // Como a função é estática, precisamos converter o parâmetro de volta para a nossa classe
    Gerenciador_MQTT* gerenciador = (Gerenciador_MQTT*)pvParameters;

    // Loop infinito da Thread (equivalente ao loop() do Arduino, mas no outro núcleo)
    for (;;) { 
        // Chama a rotina de manter o MQTT vivo ou reconectar
        gerenciador->Gerenciar_MQTT();
        
        // OBRIGATÓRIO: Dar uma pequena pausa (Yield) para o Watchdog do ESP32 não reiniciar o chip.
        // Isso permite que o Core 0 respire e processe os pacotes do rádio WiFi.
        vTaskDelay(20 / portTICK_PERIOD_MS); 
    }
}
