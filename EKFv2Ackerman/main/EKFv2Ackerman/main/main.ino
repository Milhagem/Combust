/**
 * main.ino — Integração AckermanEKF + Sensor + Telemetria
 *
 * Hierarquia de includes (sem ciclos):
 *   Kalman.hpp   ← (sem deps)
 *   Sensor.hpp   ← Kalman.hpp
 *   EKF.hpp      ← (sem deps locais)
 *   Telemetria.hpp ← (sem deps locais)
 *   main.ino     ← todos acima
 *
 * Fluxo de dados:
 *   Sensor::atualizaSensores() → getMagBearing(), getGyroZ(), getVelocidadeHALL()
 *       ↓
 *   AckermanEKF::atualizaIMU()  (75 Hz)
 *       ↓
 *   Sensor::atualizaGPS()       (5 Hz, quando disponível)
 *   AckermanEKF::atualizaGPS()
 *       ↓
 *   AckermanEKF::getEstado()    → Telemetria::EnviodadosWifi()
 */

#include <Arduino.h>
#include "Kalman.hpp"
#include "Sensor.hpp"
#include "EKF.hpp"
#include "Telemetria.hpp"

// ── Configuração física do veículo ────────────────────────────
// Meça a distância real entre os eixos dianteiro e traseiro.
static constexpr float WHEELBASE_M = 0.72f;   // [m] — ajuste para seu veículo

// Constante de tempo do filtro complementar.
// 5 s: GPS corrige a deriva do magnetômetro em ~5 s.
// Reduza para 2–3 s se o GPS for mais ruidoso que o magnetômetro.
static constexpr float TC_COMPLEMENTAR = 3.0f; // [s]

// ── Pino do sensor Hall ───────────────────────────────────────
static constexpr uint8_t HALL_PIN = 34;

// ── Objetos globais ───────────────────────────────────────────
Sensor      sensor(HALL_PIN, /*habilitarHall=*/true);
AckermanEKF ekf(WHEELBASE_M, TC_COMPLEMENTAR);
Telemetria  telemetria;

// ── Temporização ─────────────────────────────────────────────
static unsigned long t_imu_last  = 0;
static unsigned long t_gps_last  = 0;
static unsigned long t_mqtt_last = 0;

static constexpr unsigned long IMU_PERIOD_MS  =  13;   // ~75 Hz
static constexpr unsigned long GPS_PERIOD_MS  = 200;   // ~5 Hz
static constexpr unsigned long MQTT_PERIOD_MS = 500;   // 2 Hz

// ── Flag do sensor Hall (minimalista para evitar stack overflow) ────
volatile bool hallPulse = false;

// ── ISR do sensor Hall (o mais simples possível) ────────────────────
void IRAM_ATTR hallISR() {
    hallPulse = true;
}

// ═════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== AckermanEKF iniciando ===");

    Wire.begin();

    // Inicializa IMU
    sensor.inicializaICM();

    // Calibra giroscópio (veículo parado, ~5 s)
    Serial.println("Mantenha o veiculo parado para calibracao do giroscopio...");
    sensor.calibrarGiroscopio(500);

    // Inicializa GPS
    sensor.inicializaGPS();

    // Configura Hall
    pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

    // Inicializa EKF
    ekf.init();

    // ── Inicializa WiFi/MQTT (não bloqueia, apenas tenta conectar) ──
    telemetria.init_wifi();
    delay(1000);  // Aguarda WiFi se bem-sucedido

    // ── Sintonização opcional (valores padrão já são razoáveis) ──
    // Se o seu GPS for melhor (ex: u-blox com SBAS): reduza R_gps_pos
    // ekf.setR("gps_pos", 1.0f);

    // Se fizer curvas bruscas: aumente Q de delta
    // ekf.setQ(8, 0.02f);

    Serial.println("Setup concluido. Aguardando fix GPS...");
}

// ═════════════════════════════════════════════════════════════
void loop() {
    unsigned long now = millis();
    // ── Processa Hall fora da ISR (evita stack overflow) ────────────
    if (hallPulse) {
        hallPulse = false;
        sensor.calc();
    }
    // ── Ciclo IMU: ~75 Hz ─────────────────────────────────────
    if (now - t_imu_last >= IMU_PERIOD_MS) {
        t_imu_last = now;

        sensor.atualizaSensores();

        float theta_imu  = sensor.getMagBearing();      // [rad] bearing calibrado
        float omega_z    = sensor.getGyroZ();            // [rad/s] yaw rate
        float speed_mps  = (float)sensor.getVelocidadeHALL(); // [m/s]

        ekf.atualizaIMU(theta_imu, omega_z, speed_mps);
    }

    // ── Ciclo GPS: ~5 Hz ─────────────────────────────────────
    if (now - t_gps_last >= GPS_PERIOD_MS) {
        t_gps_last = now;

        sensor.atualizaGPS();

        // Só atualiza o EKF se o GPS tiver fix
        if (sensor.getLatitude() != 0.0f && sensor.getLongitude() != 0.0f) {

            // Course over ground: TinyGPS++ fornece em graus, precisa converter
            // e verificar se o dado é válido (requer velocidade mínima)
            float speed_mps = (float)sensor.getVelocidadeHALL();
            float theta_gps = NAN;   // inicializa inválido

            // GPS bearing só é confiável acima de 0.5 m/s
            // TinyGPS++ não expõe course diretamente via Sensor — adapte
            // se sua versão de Sensor expuser sensor.getCourseRad().
            // Por ora, passa NAN e o EKF usa só a posição GPS.
            // Para habilitar: theta_gps = sensor.getCourseRad();

            ekf.atualizaGPS(
                sensor.getLatitude(),
                sensor.getLongitude(),
                theta_gps
            );
        }
    }

    // ── Telemetria MQTT: 2 Hz ─────────────────────────────────
    if (now - t_mqtt_last >= MQTT_PERIOD_MS) {
        t_mqtt_last = now;

        AckermanEKF::Estado e = ekf.getEstado();

        // EnviodadosWifi aceita: accel, pitch, yaw, lat, lon
        // Adaptamos: enviamos heading como "yaw", delta como "pitch",
        // e posição local como lat/lon (em metros do ponto de referência).
        // Ajuste os campos conforme seu dashboard.
        telemetria.EnviodadosWifi(

            e.speed,            // velocidade [m/s] no campo "accel"
            e.delta * 57.2957f, // esterço em graus no campo "pitch"
            e.theta * 57.2957f, // heading em graus no campo "yaw"
            e.X,                // posição Este local [m] no campo "lat"
            e.Y                 // posição Norte local [m] no campo "lon"
        );

        // Debug serial
        ekf.printSerial();
    }
}
