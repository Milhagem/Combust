#pragma once
#include <Arduino.h>
#include <math.h>

#define EKF_N 5  // Dimensão do vetor de estado: [X, Y, v, theta, omega_bias]

class Filtro_Kalman_Extendido {
private:
    // ── Estado e Covariância ───────────────────────────────
    inline static float x[EKF_N] = {0};         // Vetor de estado
    inline static float P[EKF_N * EKF_N] = {0}; // Matriz de covariância
    inline static float Q_diag[EKF_N] = {0};    // Ruído de processo (Q)
    
    // ── Ruídos de Medição (R) ──────────────────────────────
    inline static float R_gps_pos = 4.0f;       // GPS [m²]
    inline static float R_hall = 0.04f;         // Sensor Hall [(m/s)²]
    inline static float R_gps_heading = 0.1f;   // Heading via GPS (quando em movimento)
    
    // ── Variáveis de Controle e Referência ─────────────────
    inline static bool gps_ref_set = false;
    inline static double ref_lat = 0.0;
    inline static double ref_lon = 0.0;
    inline static unsigned long last_us = 0;
    
    inline static float last_ax = 0.0f;
    inline static bool veiculo_parado = false;  // Flag Start-Stop

    // ── Funções Matemáticas Internas ───────────────────────
    static float wrapAngle(float a);
    static void updateScalar(const float H[EKF_N], float innov, float R);
    static void gpsToLocal(double lat, double lon, float &X, float &Y);

public:
    // ── Inicialização ──────────────────────────────────────
    static void begin();

    // ── Ciclo Principal (Predição + Sensores Rápidos) ──────
    static void predict(float ax, float omega_z, float dt);
    static void updateHall(float speed_hall);
    
    // ── Atualização Lenta (GPS) ────────────────────────────
    static void updateGPS(double lat, double lon, float gps_speed, float gps_course);

    // ── Start-Stop Lógica ──────────────────────────────────
    static void setVeiculoParado(bool parado);

    // ── Getters ────────────────────────────────────────────
    static float getX() { return x[0]; }
    static float getY() { return x[1]; }
    static float getV() { return x[2]; }
    static float getTheta() { return x[3]; }
    static float getOmegaBias() { return x[4]; }

    // ── Sintonia em Tempo de Execução (Telemetria) ─────────
    static void setQ(int idx, float val);
    static void setR(const char* sensor, float val);
};

