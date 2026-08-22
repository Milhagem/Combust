#include "Callback.hpp"
#include "Filtro_Kalman_Extendido.hpp" // Header oficial do Filtro de Kalman Estendido

// Instância global do Filtro de Kalman Estendido declarada no código principal (main/setup)
extern FiltroKalmanExtendido ekf;

void Callback::carregarParametrosIniciais() {
    Preferences pref;
    pref.begin("configMotor", true);

    // ==========================================
    // CARREGA MOTORES
    // ==========================================
    StartStop::velocidadeMinima = pref.getFloat("vel_min", 8.0f);
    StartStop::velocidadeMax    = pref.getFloat("vel_max", 20.0f);
    StartStop::RPMideal         = pref.getFloat("rpm_alvo", 3500.0f);
    StartStop::PosBorboIdeal    = pref.getFloat("tps_alvo", 4000.0f);
    StartStop::modoControle     = pref.getInt("modo_ctrl", 0);
    
    ServoMotor::posInicialServo = pref.getInt("pos_ini", 1056);
    
    Motor::POS_SERVO_PARTIDA    = pref.getInt("pos_partida", 1000);
    Motor::POS_SERVO_FECHADA    = pref.getInt("pos_fechada", 500);

    // ==========================================
    // CARREGA EKF E APLICA NO FILTRO
    // ==========================================
    ekf.setQ(0, pref.getFloat("q_x", 0.02f));
    ekf.setQ(1, pref.getFloat("q_y", 0.02f));
    ekf.setQ(2, pref.getFloat("q_v", 0.20f));
    ekf.setQ(3, pref.getFloat("q_theta", 0.01f));
    ekf.setQ(4, pref.getFloat("q_omega", 0.0005f));
    
    ekf.setR("gps_pos", pref.getFloat("r_gps", 4.0f));
    ekf.setR("hall", pref.getFloat("r_hall", 0.04f));

    pref.end();
}

void Callback::processarMensagem(byte* payload, unsigned int length, PubSubClient& client, const char* topico_feedback) {
    JsonDocument doc; 
    
    if (deserializeJson(doc, payload, length)) { 
        client.publish(topico_feedback, "{\"status\":\"ERRO_JSON_INVALIDO\"}"); 
        return; 
    }

    Preferences pref;
    pref.begin("configMotor", false);
    bool alterou = false;

    // ==========================================
    // 1. PARÂMETROS DO MOTOR E START-STOP 
    // ==========================================
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

    // ==========================================
    // 2. PARÂMETROS DO FILTRO EKF (Usando GETTERS diretamente)
    // ==========================================
    
    // Matriz Q (Processo)
    if (doc["q_x"].is<float>()) { 
        float v = doc["q_x"].as<float>(); 
        if (v != ekf.getQ(0)) { ekf.setQ(0, v); pref.putFloat("q_x", v); alterou = true; } 
    }
    if (doc["q_y"].is<float>()) { 
        float v = doc["q_y"].as<float>(); 
        if (v != ekf.getQ(1)) { ekf.setQ(1, v); pref.putFloat("q_y", v); alterou = true; } 
    }
    if (doc["q_v"].is<float>()) { 
        float v = doc["q_v"].as<float>(); 
        if (v != ekf.getQ(2)) { ekf.setQ(2, v); pref.putFloat("q_v", v); alterou = true; } 
    }
    if (doc["q_theta"].is<float>()) { 
        float v = doc["q_theta"].as<float>(); 
        if (v != ekf.getQ(3)) { ekf.setQ(3, v); pref.putFloat("q_theta", v); alterou = true; } 
    }
    if (doc["q_omega"].is<float>()) { 
        float v = doc["q_omega"].as<float>(); 
        if (v != ekf.getQ(4)) { ekf.setQ(4, v); pref.putFloat("q_omega", v); alterou = true; } 
    }

    // Matriz R (Medição)
    if (doc["r_gps"].is<float>()) { 
        float v = doc["r_gps"].as<float>(); 
        if (v != ekf.getR("gps_pos")) { ekf.setR("gps_pos", v); pref.putFloat("r_gps", v); alterou = true; } 
    }
    if (doc["r_hall"].is<float>()) { 
        float v = doc["r_hall"].as<float>(); 
        if (v != ekf.getR("hall")) { ekf.setR("hall", v); pref.putFloat("r_hall", v); alterou = true; } 
    }

    pref.end();

    // ==========================================
    // 3. CONSTRUÇÃO DO FEEDBACK 
    // ==========================================
    const char* statusStr = alterou ? "OK_ATUALIZADO" : "SEM_ALTERACOES";
    char feedback[512]; 
    
    snprintf(feedback, sizeof(feedback),
             "{\"status\":\"%s\",\"vel_min\":%.2f,\"vel_max\":%.2f,\"rpm_alvo\":%.2f,\"tps_alvo\":%.2f,\"modo_ctrl\":%d,"
             "\"pos_ini\":%d,\"pos_partida\":%d,\"pos_fechada\":%d,"
             "\"q_v\":%.4f,\"r_gps\":%.4f,\"r_hall\":%.4f}",
             statusStr, 
             StartStop::velocidadeMinima, StartStop::velocidadeMax, StartStop::RPMideal, 
             StartStop::PosBorboIdeal, StartStop::modoControle,
             ServoMotor::posInicialServo, Motor::POS_SERVO_PARTIDA, Motor::POS_SERVO_FECHADA,
             ekf.getQ(2), ekf.getR("gps_pos"), ekf.getR("hall"));

    client.publish(topico_feedback, feedback);
}