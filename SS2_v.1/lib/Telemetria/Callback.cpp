#include "Callback.hpp"
#include "Filtro_Kalman_Extendido.hpp" // Header oficial do Filtro de Kalman Estendido

// Instância global do Filtro de Kalman Estendido declarada no código principal (main/setup)
extern FiltroKalmanExtendido ekf;

void Callback::carregarParametrosIniciais() {
    Preferences pref;
    pref.begin("configMotor", true);

    StartStop::velocidadeMinima = pref.getFloat("vel_min", 8.0f);
    StartStop::velocidadeMax    = pref.getFloat("vel_max", 20.0f);
    StartStop::RPMideal         = pref.getFloat("rpm_alvo", 3500.0f);
    StartStop::PosBorboIdeal    = pref.getFloat("tps_alvo", 4000.0f);
    StartStop::modoControle     = pref.getInt("modo_ctrl", 0);
    
    ServoMotor::posInicialServo = pref.getInt("pos_ini", 1056);
    
    Motor::POS_SERVO_PARTIDA    = pref.getInt("pos_partida", 1000);
    Motor::POS_SERVO_FECHADA    = pref.getInt("pos_fechada", 500);

    pref.end();
}

void Callback::processarMensagem(byte* payload, unsigned int length, PubSubClient& client, const char* topico_feedback) {
    JsonDocument doc; 
    
    if (deserializeJson(doc, payload, length)) { 
        client.publish(topico_feedback, "{\"status\":\"ERRO_JSON_INVALIDO\"}"); 
        return; 
    }

    // ==========================================
    // 3. PARÂMETROS DO MOTOR E START-STOP (Lógica Original)
    // ==========================================
    Preferences pref;
    pref.begin("configMotor", false);
    bool alterou = false;

    if (doc["vel_min"].is<float>()) { 
        float v = doc["vel_min"].as<float>(); 
        if (v != StartStop::velocidadeMinima) { StartStop::velocidadeMinima = v; pref.putFloat("vel_min", v); alterou = true; } 
    }
    if (doc["vel_max"].is<float>()) { 
        float v = doc["vel_max"].as<float>(); 
        if (v != StartStop::velocidadeMax) { StartStop::velocidadeMax = v; pref.putFloat("vel_max", v); alterou = true; } 
    }
    if (doc["rpm_alvo"].is<float>()) { 
        float v = doc["rpm_alvo"].as<float>(); 
        if (v != StartStop::RPMideal) { StartStop::RPMideal = v; pref.putFloat("rpm_alvo", v); alterou = true; } 
    }
    if (doc["tps_alvo"].is<float>()) { 
        float v = doc["tps_alvo"].as<float>(); 
        if (v != StartStop::PosBorboIdeal) { StartStop::PosBorboIdeal = v; pref.putFloat("tps_alvo", v); alterou = true; } 
    }
    if (doc["modo_ctrl"].is<int>()) { 
        int v = doc["modo_ctrl"].as<int>(); 
        if (v != StartStop::modoControle) { StartStop::modoControle = v; pref.putInt("modo_ctrl", v); alterou = true; } 
    }

    if (doc["pos_ini"].is<int>()) { 
        int v = doc["pos_ini"].as<int>(); 
        if (v != ServoMotor::posInicialServo) { ServoMotor::posInicialServo = v; pref.putInt("pos_ini", v); alterou = true; } 
    }

    if (doc["pos_partida"].is<int>()) { 
        int v = doc["pos_partida"].as<int>(); 
        if (v != Motor::POS_SERVO_PARTIDA) { Motor::POS_SERVO_PARTIDA = v; pref.putInt("pos_partida", v); alterou = true; } 
    }
    if (doc["pos_fechada"].is<int>()) { 
        int v = doc["pos_fechada"].as<int>(); 
        if (v != Motor::POS_SERVO_FECHADA) { Motor::POS_SERVO_FECHADA = v; pref.putInt("pos_fechada", v); alterou = true; } 
    }

    pref.end();

    // ==========================================
    // 4. CONSTRUÇÃO DO FEEDBACK DO MOTOR
    // ==========================================
    const char* statusStr = alterou ? "OK_ATUALIZADO" : "SEM_ALTERACOES";
    
    char feedback[350]; 
    
    snprintf(feedback, sizeof(feedback),
             "{\"status\":\"%s\",\"vel_min\":%.2f,\"vel_max\":%.2f,\"rpm_alvo\":%.2f,\"tps_alvo\":%.2f,\"modo_ctrl\":%d,\"pos_ini\":%d,\"pos_partida\":%d,\"pos_fechada\":%d}",
             statusStr, 
             StartStop::velocidadeMinima,
             StartStop::velocidadeMax,
             StartStop::RPMideal, 
             StartStop::PosBorboIdeal, 
             StartStop::modoControle,
             ServoMotor::posInicialServo,
             Motor::POS_SERVO_PARTIDA,
             Motor::POS_SERVO_FECHADA);

    client.publish(topico_feedback, feedback);
}