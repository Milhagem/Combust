#ifndef EKF_HPP
#define EKF_HPP

/**
 * ============================================================
 * Extended Kalman Filter — Modelo Ackerman (Weinstein & Moore, 2010)
 * Adaptado para ESP32-S3 com ICM20948 + GPS NMEA + Sensor Hall
 *
 * Vetor de estado (9D):
 *   x[0] = X        — posição local Este         [m]   (coordenadas locais ENU)
 *   x[1] = Y        — posição local Norte        [m]
 *   x[2] = theta    — heading verdadeiro         [rad]
 *   x[3] = theta_B  — bias magnetômetro/IMU      [rad]  (passeio aleatório)
 *   x[4] = x_B      — bias GPS Este              [m]   (passeio aleatório)
 *   x[5] = y_B      — bias GPS Norte             [m]   (passeio aleatório)
 *   x[6] = x_hp     — estado interno passa-alta  [rad]  (filtro complementar)
 *   x[7] = x_lp     — estado interno passa-baixa [rad]  (filtro complementar)
 *   x[8] = delta    — ângulo de esterço estimado [rad]  (passeio aleatório)
 *
 * Modelo cinemático Ackerman (equações 1a-1c do artigo):
 *   Xdot    =  v * cos(pi/2 - theta)
 *   Ydot    =  v * sin(pi/2 - theta)
 *   thetadot=  (v / L) * tan(delta)
 *
 * Filtro complementar:
 *   theta_IMU → passa-alta (capta dinâmica rápida)
 *   theta_GPS → passa-baixa (corrige deriva lenta)
 *   theta_fused = x_hp + x_lp
 *
 * Estimativa de delta (ângulo de esterço) — Abordagem 2+3:
 *   - delta é estado do EKF (passeio aleatório)
 *   - giroscópio Z é observação: omega_z ≈ (v/L)*tan(delta)
 *   - jacobiano ∂thetadot/∂delta = (v/L)/cos²(delta) propaga delta→theta
 *
 * Matrizes H adaptativas (Seção III do artigo):
 *   H_FULL         : GPS pos + GPS bearing + IMU bearing + IMU+GPS fused + gyro Z
 *   H_NO_GPS_BEARING: sem bearing GPS (v < 0.5 m/s)
 *   H_IMU_ONLY     : só IMU disponível
 * ============================================================
 */

#include <Arduino.h>
#include <math.h>

#define AEKF_N          9      // dimensão do vetor de estado
#define AEKF_MIN_SPEED_GPS 0.5f  // velocidade mínima para usar bearing GPS [m/s]

class AckermanEKF {
public:

    // ─────────────────────────────────────────────────────────
    //  Resultado público
    // ─────────────────────────────────────────────────────────
    struct Estado {
        float X;            // posição Este  [m]
        float Y;            // posição Norte [m]
        float theta;        // heading       [rad]
        float delta;        // esterço estim.[rad]
        float theta_fused;  // bearing fusionado (filtro complementar) [rad]
        float speed;        // velocidade corrente [m/s]
    };

    // ─────────────────────────────────────────────────────────
    //  Construção e configuração
    // ─────────────────────────────────────────────────────────

    /**
     * @param wheelbase_m   Distância entre eixos dianteiro e traseiro [m]
     * @param Tc_s          Constante de tempo do filtro complementar  [s]
     *                      Maior Tc → mais peso no GPS para corrigir deriva;
     *                      sugestão inicial: 5.0 s
     */
    explicit AckermanEKF(float wheelbase_m, float Tc_s = 5.0f);

    /** Reinicializa estado, covariância e referência GPS. */
    void init();

    // ─────────────────────────────────────────────────────────
    //  Interface de atualização
    // ─────────────────────────────────────────────────────────

    /**
     * Ciclo principal — chamar a cada leitura do ICM20948 (~75 Hz).
     *
     * @param theta_imu   Bearing magnético calibrado e corrigido    [rad]
     *                    (saída de atan2(my_cal, mx_cal) + declinação)
     * @param omega_z     Giroscópio Z com bias já removido          [rad/s]
     * @param speed_mps   Velocidade do sensor Hall                  [m/s]
     */
    void atualizaIMU(float theta_imu, float omega_z, float speed_mps);

    /**
     * Atualização GPS — chamar apenas quando chegar nova posição válida (~5 Hz).
     *
     * @param lat         Latitude                                   [°]
     * @param lon         Longitude                                  [°]
     * @param theta_gps   Course over ground em radianos             [rad]
     *                    Passe NAN quando v < 0.5 m/s ou indisponível
     */
    void atualizaGPS(double lat, double lon, float theta_gps = NAN);

    /** Retorna cópia do estado atual. */
    Estado getEstado() const;

    /** Imprime estado via Serial (debug). */
    void printSerial() const;

    // ─────────────────────────────────────────────────────────
    //  Sintonização em tempo de execução
    // ─────────────────────────────────────────────────────────

    /**
     * Ajusta elemento diagonal de Q (ruído de processo).
     * @param idx  Índice do estado (0..8)
     * @param val  Novo valor [unidade² / s]
     */
    void setQ(int idx, float val);

    /**
     * Ajusta ruídos de medição R.
     * @param sensor "imu_bearing" | "gps_pos" | "gps_bearing" | "gyro_z"
     * @param val    Novo valor [unidade²]
     */
    void setR(const char *sensor, float val);

private:
    // ─────────────────────────────────────────────────────────
    //  Estado e parâmetros
    // ─────────────────────────────────────────────────────────
    float x[AEKF_N];           // vetor de estado estimado
    float P[AEKF_N * AEKF_N];  // matriz de covariância (row-major)
    float Q_diag[AEKF_N];      // diagonal de Q

    float R_imu;               // σ² bearing IMU    [rad²]
    float R_gps_pos;           // σ² posição GPS    [m²]
    float R_gps_bearing;       // σ² bearing GPS    [rad²]
    float R_gyro;              // σ² giroscópio Z   [(rad/s)²]

    float L;                   // wheelbase         [m]
    float Tc;                  // cte filtro compl. [s]
    float v_current;           // velocidade atual  [m/s]

    bool   gps_ref_set;
    double ref_lat, ref_lon;
    unsigned long last_us;

    // ─────────────────────────────────────────────────────────
    //  Passos internos do EKF
    // ─────────────────────────────────────────────────────────

    // Predição: propaga x e P pelo modelo Ackerman + bias + filtro compl.
    void predict(float dt, float theta_imu, float theta_gps_lp);

    // Atualizações individuais por sensor
    // Cada uma monta sua linha(s) de H, vetor z e chama updateCore
    void updateIMUbearing (float theta_imu);
    void updateFusedBearing(float theta_fused_meas);
    void updateGPSpos     (float gX, float gY);
    void updateGPSbearing (float theta_gps);
    void updateGyroZ      (float omega_z);

    /**
     * Núcleo genérico de atualização EKF para nz observações escalares
     * independentes (R diagonal — evita inversão de matriz grande).
     *
     * Processa cada linha separadamente: S_i escalar, K_i vetor.
     * Custo: O(nz * N²) ao invés de inversão nz×nz.
     *
     * @param z       Vetor de medições                    [nz]
     * @param h       Predição do modelo para cada medição [nz]
     * @param H       Jacobiano (nz × N) row-major         [nz * AEKF_N]
     * @param R_diag  Diagonal de R (variancias)           [nz]
     * @param nz      Número de observações
     */
    void updateCore(const float *z, const float *h,
                    const float *H, const float *R_diag, int nz);

    // Converte lat/lon para coordenadas locais ENU
    void gpsToLocal(double lat, double lon, float &X, float &Y) const;

    // Normaliza ângulo para [-pi, pi]
    static float wrapAngle(float a);

    // ─────────────────────────────────────────────────────────
    //  Álgebra linear (sem alocação dinâmica)
    //  Todas as matrizes row-major, tamanho passado por parâmetro.
    // ─────────────────────────────────────────────────────────
    static void mat_zero (float *A, int n);
    static void mat_eye  (float *A, int n);
    static void mat_copy (const float *src, float *dst, int n);
    static void mat_add  (const float *A, const float *B, float *C, int n);
    static void mat_sub  (const float *A, const float *B, float *C, int n);

    // C[m×n] = A[m×k] * B[k×n]
    static void mat_mul  (const float *A, const float *B, float *C,
                          int m, int k, int n);
    // C[m×n] = A[m×k] * B[n×k]^T
    static void mat_mulT (const float *A, const float *B, float *C,
                          int m, int k, int n);
};

#endif // EKF_HPP
