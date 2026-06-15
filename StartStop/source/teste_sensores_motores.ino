#include <WiFi.h>
#include <PubSubClient.h>
#include "BSFC.hpp" 
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Preferences.h>   
#include <ArduinoJson.h>

// =============================================================
// CONFIGURAÇÕES DE HARDWARE E SD
// =============================================================
#define SD_MISO 11
#define SD_MOSI 12
#define SD_SCK  13
#define SD_CS   4
#define PIN_SERVO 47
#define BOTAO_BOOT 0

bool sdOnline = false;
char currentFileName[32] = ""; // Substituição de String por Array fixo
char bufferSD[2048] = "";      // Fim da fragmentação (Buffer fixo de 2KB)
int contagemBuffer = 0;

// =============================================================
// CONFIGURAÇÕES WI-FI E MQTT
// =============================================================
const char* ssid = "Urban";
const char* password = "1819201234";
const char* mqtt_server = "maqiatto.com";
const int mqtt_port = 1883;
const char* mqtt_user = "ricardofonsecaj123@gmail.com";
const char* mqtt_password = "12345678";

const char* topico_telemetria = "ricardofonsecaj123@gmail.com/teste";
const char* topico_config = "ricardofonsecaj123@gmail.com/telemetria";
const char* topico_feedback = "ricardofonsecaj123@gmail.com/feedback"; 

WiFiClient espClient;
PubSubClient client(espClient);

// =============================================================
// OBJETOS E VARIÁVEIS GLOBAIS
// =============================================================
Bsfc motor;

// Variáveis do Controlador PI
String modoCtrl = "0";    
float rpmAlvo = 3000.0;
float tpsAlvo = 10.0;    
float kpTun = 0.0;       
float kiTun = 4.5;       
int passoMaxTun = 31;
int posInicialServo = 1056;

// Variáveis do Filtro de Kalman
float kMea = 20.0; 
float kEst = 20.0;
float kQ   = 0.1;  

// Variáveis de trabalho do servo
long servoInterval = 100;
int pulsoMin = 500;
int pulsoMax = 2400;
int pulsoServo = 1056;
const float histerese = 0.6;
unsigned long lastServoTime = 0;

// Controle de Tempo
unsigned long lastLogTime = 0;
const long logInterval = 100;
unsigned long lastMqttTime = 0;
const long mqttInterval = 500; 
unsigned long lastStatusTime = 0;
const long statusInterval = 2000;
unsigned long lastReconnectAttempt = 0;

// =============================================================
// FUNÇÃO NATIVA DO SERVO
// =============================================================
void escreveServo(int microssegundos) {
    if(microssegundos < 500) microssegundos = 500;
    if(microssegundos > 2500) microssegundos = 2500;
    uint32_t duty = (uint32_t)( ((uint64_t)microssegundos * 16384ULL) / 20000ULL );
    ledcWrite(PIN_SERVO, duty);
}

// =============================================================
// SETUP WI-FI E SD
// =============================================================
void setupSD() {
    pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SPI, 4000000)) { sdOnline = false; return; }
    
    int fileNumber = 0;
    while (true) {
        snprintf(currentFileName, sizeof(currentFileName), "/log%d.csv", fileNumber);
        if (!SD.exists(currentFileName)) break;
        fileNumber++; yield(); 
    }
    
    File file = SD.open(currentFileName, FILE_WRITE);
    if (file) { file.close(); sdOnline = true; } else { sdOnline = false; }
}

void setup_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true); // Driver nativo passa a gerenciar quedas ocultas
    WiFi.begin(ssid, password);
    int tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 20) { delay(500); tentativas++; }
}

// =============================================================
// CALLBACK MQTT (Tuning Remoto)
// =============================================================
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, topico_config) == 0) {
        StaticJsonDocument<512> doc;
        if (deserializeJson(doc, payload, length)) { client.publish(topico_feedback, "{\"status\":\"ERRO_JSON\"}"); return; }

        Preferences pref;
        pref.begin("configMotor", false);
        bool alterou = false;

        if (doc.containsKey("modo_ctrl")) { String v = doc["modo_ctrl"].as<String>(); if (v != modoCtrl) { modoCtrl = v; pref.putString("modo_ctrl", v); alterou = true; } }
        if (doc.containsKey("rpm_alvo")) { float v = doc["rpm_alvo"]; if (v != rpmAlvo) { rpmAlvo = v; pref.putFloat("rpm_alvo", v); alterou = true; } }
        if (doc.containsKey("tps_alvo")) { float v = doc["tps_alvo"]; if (v != tpsAlvo) { tpsAlvo = v; pref.putFloat("tps_alvo", v); alterou = true; } }
        if (doc.containsKey("kp")) { float v = doc["kp"]; if (v != kpTun) { kpTun = v; pref.putFloat("kp", v); alterou = true; } }
        if (doc.containsKey("ki")) { float v = doc["ki"]; if (v != kiTun) { kiTun = v; pref.putFloat("ki", v); alterou = true; } }
        if (doc.containsKey("passo_max")) { int v = doc["passo_max"]; if (v != passoMaxTun) { passoMaxTun = v; pref.putInt("passo_max", v); alterou = true; } }
        if (doc.containsKey("pos_ini")) { int v = doc["pos_ini"]; if (v != posInicialServo) { posInicialServo = v; pref.putInt("pos_ini", v); alterou = true; } }

        if (doc.containsKey("k_mea")) { float v = doc["k_mea"]; if (v != kMea) { kMea = v; pref.putFloat("k_mea", v); alterou = true; } }
        if (doc.containsKey("k_est")) { float v = doc["k_est"]; if (v != kEst) { kEst = v; pref.putFloat("k_est", v); alterou = true; } }
        if (doc.containsKey("k_q"))   { float v = doc["k_q"];   if (v != kQ)   { kQ = v;   pref.putFloat("k_q", v);   alterou = true; } }

        pref.end();

        if (alterou) motor.atualizaKalman(kMea, kEst, kQ);

        const char* statusStr = alterou ? "OK" : "SEM_ALTERACOES";
        
        // Uso de snprintf no lugar de concatenação com '+' para preservar memória
        char feedback[350];
        snprintf(feedback, sizeof(feedback),
                 "{\"status\":\"%s\",\"modo_ctrl\":\"%s\",\"rpm_alvo\":%.2f,\"tps_alvo\":%.2f,\"kp\":%.2f,\"ki\":%.2f,\"passo_max\":%d,\"pos_ini\":%d,\"k_mea\":%.2f,\"k_est\":%.2f,\"k_q\":%.3f}",
                 statusStr, modoCtrl.c_str(), rpmAlvo, tpsAlvo, kpTun, kiTun, passoMaxTun, posInicialServo, kMea, kEst, kQ);

        client.publish(topico_feedback, feedback);
    }
}

boolean reconnect() {
    if (WiFi.status() != WL_CONNECTED) return false; // Não briga com a reconexão nativa do Wi-Fi

    if (client.connect("ESP32_BSFC_Client", mqtt_user, mqtt_password)) { 
        client.subscribe(topico_config); 
        return true; 
    }
    return false;
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    
    pinMode(BOTAO_BOOT, INPUT_PULLUP);
    delay(3000);

    Preferences prefSetup;
    prefSetup.begin("configMotor", false);
    
    // Hard Reset via botão BOOT
    if (digitalRead(BOTAO_BOOT) == LOW) {
        Serial.println("\n⚠️ RESET DE EMERGENCIA ATIVADO!");
        prefSetup.clear(); 
    } else {
        modoCtrl = prefSetup.getString("modo_ctrl", modoCtrl);
        rpmAlvo = prefSetup.getFloat("rpm_alvo", rpmAlvo);
        tpsAlvo = prefSetup.getFloat("tps_alvo", tpsAlvo);
        kpTun = prefSetup.getFloat("kp", kpTun);
        kiTun = prefSetup.getFloat("ki", kiTun);
        passoMaxTun = prefSetup.getInt("passo_max", passoMaxTun);
        posInicialServo = prefSetup.getInt("pos_ini", posInicialServo);
        
        kMea = prefSetup.getFloat("k_mea", kMea);
        kEst = prefSetup.getFloat("k_est", kEst);
        kQ   = prefSetup.getFloat("k_q", kQ);
    }
    prefSetup.end();
    
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callbackMQTT);
    
    // Travas anti-congelamento de Socket TCP do MQTT
    client.setSocketTimeout(1); 
    client.setBufferSize(512);

    setupSD();
    motor.init();
    motor.atualizaKalman(kMea, kEst, kQ);

    pinMode(PIN_SERVO, OUTPUT);
    ledcAttach(PIN_SERVO, 50, 14);
    pulsoServo = posInicialServo;
    escreveServo(pulsoServo);
}

// =============================================================
// LOOP PRINCIPAL
// =============================================================
void loop() {
    if (!client.connected()) {
        unsigned long now = millis();
        if (now - lastReconnectAttempt > 5000) { 
            lastReconnectAttempt = now; 
            if (reconnect()) lastReconnectAttempt = 0; 
        }
    } else { 
        client.loop(); 
    }

    motor.analisaMap();
    motor.analisaPosBorbo();
    motor.analisaLambda(); 
    motor.analisaRPM();

    unsigned long currentMillis = millis();

    // =========================================================
    // CONTROLE PI INCREMENTAL
    // =========================================================
    if (currentMillis - lastServoTime >= servoInterval) {
        lastServoTime = currentMillis;
        float tpsAtual = motor.getPosBorbo();
        float rpmAtual = motor.getRpm();
        int leituraBrutaTps = analogRead(Pintp); 
        float erro = 0.0;
        static float erroAnterior = 0.0; 

        if (rpmAtual > 300 && leituraBrutaTps > 100) {
            if (modoCtrl == "0") erro = tpsAlvo - tpsAtual;
            else erro = (rpmAlvo - rpmAtual) * 0.02; 
            
            if (abs(erro) > histerese) {
                float termoP = kpTun * (erro - erroAnterior);
                float termoI = kiTun * erro;                
                int compensacao = (int)(termoP + termoI);
                
                if (compensacao > passoMaxTun) compensacao = passoMaxTun;
                if (compensacao < -passoMaxTun) compensacao = -passoMaxTun;

                pulsoServo += compensacao;
                if (pulsoServo < pulsoMin) pulsoServo = pulsoMin;
                if (pulsoServo > pulsoMax) pulsoServo = pulsoMax;
            }
            erroAnterior = erro;
        } else {
            pulsoServo = posInicialServo; 
            erroAnterior = 0.0;
        }
        escreveServo(pulsoServo);
    }

   // =========================================================
    // GRAVAÇÃO SD E MONITOR SERIAL (Modo Limpo)
    // =========================================================
    if (currentMillis - lastLogTime >= logInterval) {
        lastLogTime = currentMillis;
        
        // --- MONITOR SERIAL (Apenas Essencial) ---
        Serial.print("MAP: "); Serial.print(analogRead(pinmap));
        Serial.print(" | TPS(%): "); Serial.print(motor.getPosBorbo());
        Serial.print(" | RPM(Bruto): "); Serial.print(Bsfc::rpm_calculado);
        Serial.print(" | RPM(Filtro): "); Serial.println(motor.getRpm());

        // --- GRAVAÇÃO NO SD ---
        // CSV: Tempo(ms), MAP, TPS(%), RPM_Bruto, RPM_Filtro
        char linha[80];
        snprintf(linha, sizeof(linha), "%lu,%d,%.2f,%lu,%.2f\n", 
                 currentMillis, 
                 analogRead(pinmap), 
                 motor.getPosBorbo(), 
                 Bsfc::rpm_calculado, 
                 motor.getRpm());
        
        strlcat(bufferSD, linha, sizeof(bufferSD));
        contagemBuffer++;

        if (contagemBuffer >= 20) {
            if (sdOnline) {
                File dataFile = SD.open(currentFileName, FILE_APPEND);
                if (dataFile) { 
                    dataFile.print(bufferSD); 
                    dataFile.close(); 
                } else {
                    Serial.println("⚠️ ERRO: SD Card falhou! (Vibracao?)");
                }
            }
            bufferSD[0] = '\0'; 
            contagemBuffer = 0;
        }
    }

    // =========================================================
    // ENVIO MQTT
    // =========================================================
    if (client.connected() && (currentMillis - lastMqttTime >= mqttInterval)) {
        lastMqttTime = currentMillis;
        
        char payload[200];
        snprintf(payload, sizeof(payload), 
                 "{\"rpm\":%.2f,\"map\":%d,\"lambda\":%.2f,\"tps\":%.2f,\"servo_atual\":%d}",
                 motor.getRpm(), analogRead(pinmap), motor.getLambda(), motor.getPosBorbo(), pulsoServo);
                 
        client.publish(topico_telemetria, payload);
    }

    // =========================================================
    // STATUS DE CONEXÃO (A cada 2 segundos)
    // =========================================================
    if (currentMillis - lastStatusTime >= statusInterval) {
        lastStatusTime = currentMillis;
        Serial.print("[STATUS] WiFi: ");
        Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "ERRO");
        Serial.print(" | MQTT: ");
        Serial.print(client.connected() ? "OK" : "ERRO");
        Serial.print(" | Arquivo SD: ");
        Serial.println(sdOnline ? currentFileName : "OFF");
    }
}
