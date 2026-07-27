#ifndef FILTRO_KALMAN_EXTENDIDO_HPP
#define FILTRO_KALMAN_EXTENDIDO_HPP

/**
 * ============================================================
 * Extended Kalman Filter — Modelo "bicicleta" simplificado
 * ESP32-S3 + ICM20948 (já calibrado/filtrado em Sensor) +
 * GPS NMEA + Sensor Hall
 *
 * Vetor de estado (5D), em coordenadas cartesianas locais (ENU):
 *   x[0] = X         — posição Leste  [m]
 *   x[1] = Y         — posição Norte [m]
 *   x[2] = v         — velocidade longitudinal [m/s]
 *   x[3] = theta     — heading [rad]
 *   x[4] = omega_bias— bias residual do giroscópio Z [rad/s]
 *
 * Modelo de predição (entradas: ax do IMU, omega_z do IMU):
 *   X'     = X + v*cos(theta)*dt
 *   Y'     = Y + v*sin(theta)*dt
 *   v'     = v + ax*dt
 *   theta' = theta + (omega_z - omega_bias)*dt
 *   omega_bias' = omega_bias   (passeio aleatório, drift lento)
 *
 * Correções (todas escalares e lineares em h(x) — sem matriz H 2D
 * exceto GPS, que é tratado como duas atualizações escalares
 * independentes):
 *   - Hall:  z = v_roda            -> H = [0,0,1,0,0]
 *   - Mag :  z = theta_mag         -> H = [0,0,0,1,0]  (wrap angular)
 *   - GPS :  z = [X_gps, Y_gps]    -> H_x=[1,0,0,0,0], H_y=[0,1,0,0,0]
 * ============================================================
 */


 #include <Arduino.h>
#include <math.h>
#include <string.h>

#define EKF_N 5  // Dimensão do vetor de estado

class Filtro_Kalman_Extendido {
public:
    // ─────────────────────────────────────────────────────────
    //  Resultado público
    // ─────────────────────────────────────────────────────────
    struct Estado {
        float X;          // posição Este  [m]
        float Y;          // posição Norte [m]
        float v;          // velocidade    [m/s]
        float theta;      // heading       [rad]
        float omega_bias; // bias do giro Z [rad/s]
        float ax;         // aceleração longitudinal usada na última predição [m/s²]
    };

    // ─────────────────────────────────────────────────────────
    //  Construção e configuração
    // ─────────────────────────────────────────────────────────
    Filtro_Kalman_Extendido();

    // Reinicializa estado, covariância e referência GPS.
    void init();

    // ─────────────────────────────────────────────────────────
    //  Interface de atualização
    // ─────────────────────────────────────────────────────────

    /**
     * Ciclo principal — chamar a cada leitura do ICM20948 (~75 Hz).
     *
     * @param theta_mag   Bearing magnético calibrado [rad]
     * @param omega_z     Giroscópio Z, bias de fábrica já removido [rad/s]
     * @param ax          Aceleração longitudinal (frame do veículo) [m/s²]
     * @param speed_hall  Velocidade do sensor Hall [m/s] (>=0; sempre passada,
     *                     a correção só é aplicada se válida, ver atualizaHall)
     */
    void atualizaIMU(float theta_mag, float omega_z, float ax, float speed_hall);

    /**
     * Atualização GPS — chamar apenas quando chegar nova posição válida (~5 Hz).
     *
     * @param lat  Latitude  [°]
     * @param lon  Longitude [°]
     */
    void atualizaGPS(double lat, double lon);

    // Retorna cópia do estado atual.
    Estado getEstado() const;

    // Imprime estado via Serial (debug).
    void printSerial() const;

    // ─────────────────────────────────────────────────────────
    //  Sintonização em tempo de execução
    // ─────────────────────────────────────────────────────────

    /**
     * Ajusta elemento diagonal de Q (ruído de processo).
     * @param idx  Índice do estado (0..4)
     * @param val  Novo valor [unidade² / s]
     */
    void setQ(int idx, float val);

    /**
     * Ajusta ruídos de medição R.
     * @param sensor "mag" | "gps_pos" | "hall"
     * @param val    Novo valor [unidade²]
     */
    void setR(const char *sensor, float val);

private:
    // ─────────────────────────────────────────────────────────
    //  Estado e parâmetros
    // ─────────────────────────────────────────────────────────
    float x[EKF_N];            // vetor de estado estimado
    float P[EKF_N * EKF_N];    // matriz de covariância (row-major)
    float Q_diag[EKF_N];       // diagonal de Q (por segundo)

    float R_mag;               // σ² bearing magnetômetro [rad²]
    float R_gps_pos;            // σ² posição GPS [m²]
    float R_hall;              // σ² velocidade Hall [m²/s²] (nominal)

    bool   gps_ref_set;
    double ref_lat, ref_lon;
    unsigned long last_us;

    float last_ax;             // último ax usado (para telemetria)

    // ─────────────────────────────────────────────────────────
    //  Passos internos
    // ─────────────────────────────────────────────────────────
    void predict(float ax, float omega_z, float dt);

    void updateHeading (float theta_mag);
    void updateWheelSpeed(float speed_hall);
    void updateGPSpos  (float gX, float gY);

    /**
     * Núcleo genérico de atualização EKF para 1 observação escalar.
     * @param H       Jacobiano (1 x EKF_N)
     * @param innov   Inovação z - h(x) (já com wrap angular se aplicável)
     * @param R       Variância da medição
     */
    void updateScalar(const float H[EKF_N], float innov, float R);

    // Converte lat/lon para coordenadas locais ENU (referência = 1º fix)
    void gpsToLocal(double lat, double lon, float &X, float &Y) const;

    // Normaliza ângulo para [-pi, pi]
    static float wrapAngle(float a);
};

#endif
