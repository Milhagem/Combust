/**
 * main_teste.cpp
 * =============================================================
 * Teste de integração: Telemetria + Velocidade + EKF
 * Hardware: ESP32-S3 WROOM-1
 *
 * Para rodar sem o sensor Hall conectado:
 *   Defina HALL_CONECTADO como false abaixo.
 *   O EKF continuará operando usando apenas ICM e GPS.
 *
 * Pinagem:
 *   Sensor Hall → pino 4 (interrupção, somente se HALL_CONECTADO=true)
 *   GPS         → RX=17, TX=16  (definidos em Telemetria.hpp)
 *   ICM20948    → I2C padrão (SDA/SCL do ESP32-S3)
 * =============================================================
 */

#include <Arduino.h>
#include "Telemetria.hpp"
#include "Velocidade.hpp"
#include "EKF.hpp"

// ── Configuração de hardware ──────────────────────────────────
#define PINO_HALL      4
#define HALL_CONECTADO false   // mude para true quando o sensor estiver montado

// ── Instâncias globais ──────────────────────────────────────── 
Telemetria telemetria;
Velocidade velocidade(PINO_HALL, HALL_CONECTADO);
EKF        ekf;

// ── ISR do sensor Hall ────────────────────────────────────────
void IRAM_ATTR hallISR() {
    velocidade.calc();   // no-op automático se HALL_CONECTADO=false
}

// ── Temporização ──────────────────────────────────────────────
static unsigned long tLoop  = 0;
static unsigned long tPrint = 0;
static unsigned long tGPS   = 0;
static constexpr unsigned long GPS_MIN_INTERVAL_MS = 200;

// ── setup() ───────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println(" EKF Telemetria — ESP32-S3  [TESTE]");
    Serial.println("========================================\n");

    Wire.begin();
    telemetria.inicializaICM();
    Serial.println("[OK] ICM20948 inicializado.");

    telemetria.inicializaGPS();
    Serial.println("[OK] GPS serial iniciado (9600 baud).");

    // Configura pino e interrupção independente do flag —
    // INPUT_PULLUP mantém o pino estável mesmo sem sensor.
    // calc() retorna imediatamente (no-op) se Hall desabilitado.
    pinMode(PINO_HALL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PINO_HALL), hallISR, RISING);

    if (velocidade.isHallHabilitado()) {
        Serial.println("[OK] Sensor Hall habilitado (pino " + String(PINO_HALL) + ").");
    } else {
        Serial.println("[--] Sensor Hall DESABILITADO — EKF usa apenas ICM + GPS.");
    }

    ekf.init();
    Serial.println("[OK] EKF inicializado.\n");
    Serial.println("--- Iniciando loop ---\n");

    tLoop = tPrint = tGPS = millis();
}

// ── loop() ────────────────────────────────────────────────────
void loop() {
    unsigned long agora = millis();

    if (agora - tLoop < 10) return;   // ~100 Hz
    tLoop = agora;

    // 1. IMU
    telemetria.atualizaSensores();

    float accel_g   = telemetria.getAccelLongitudinal();
    float accel_ms2 = Velocidade::gParaMs2(accel_g);
    float psi_rad   = telemetria.getYaw();

    // 2. Hall — retorna 0 automaticamente se desabilitado
    float speed_hall = (float)velocidade.getVelocidadeHALL();

    // 3. EKF: predição + Hall (se habilitado) + ICM
    ekf.atualiza(speed_hall, accel_ms2, psi_rad);

    // 4. GPS
    telemetria.atualizaGPS();
    float lat = telemetria.getLatitude();
    float lon = telemetria.getLongitude();

    bool gps_valido = (lat != 0.0f || lon != 0.0f);
    if (gps_valido && (agora - tGPS) >= GPS_MIN_INTERVAL_MS) {
        ekf.atualizaGPS((double)lat, (double)lon);
        tGPS = agora;
    }

    // 5. Serial ~5 Hz
    if (agora - tPrint < 200) return;
    tPrint = agora;

    EKF::Estado e = ekf.getEstado();
/*
    Serial.println("--- Sensores Brutos ---");
    if (velocidade.isHallHabilitado()) {
        Serial.printf("  Hall:  %.3f m/s  (%.1f km/h)\n",
                      speed_hall, speed_hall * 3.6f);
    } else {
        Serial.println("  Hall:  [desabilitado]");
    }
    Serial.printf("  ICM:   accel=%.3f g  (%.3f m/s2)  psi=%.3f rad  (%.1f deg)\n",
                  accel_g, accel_ms2, psi_rad, psi_rad * RAD2DEG);
    Serial.printf("  GPS:   lat=%.6f  lon=%.6f  %s\n",
                  lat, lon, gps_valido ? "VALIDO" : "aguardando fix...");

    Serial.println("--- EKF Estado ---");
    Serial.printf("  Posicao:    X=%7.2f m   Y=%7.2f m\n", e.X, e.Y);
    Serial.printf("  Velocidade: |V|=%5.2f m/s  (Vx=%5.2f  Vy=%5.2f)\n",
                  e.speed, e.Vx, e.Vy);
    Serial.printf("  Aceleracao: |a|=%5.2f m/s2 (ax=%5.2f  ay=%5.2f)\n",
                  e.accel, e.ax, e.ay);
    Serial.printf("  Yaw (psi):  %.4f rad  /  %.2f deg\n",
                  e.psi, e.psi * RAD2DEG);
    Serial.printf("  Cov diag:   P_X=%.4f  P_Y=%.4f  P_Vx=%.4f  P_Vy=%.4f\n",
                  ekf.P[0 * EKF_N + 0], ekf.P[1 * EKF_N + 1],
                  ekf.P[2 * EKF_N + 2], ekf.P[3 * EKF_N + 3]);
    Serial.println();*/
    

    Serial.printf("%7.2f, %7.2f\n", e.X, e.Y);
}