#ifndef EKF_HPP
#define EKF_HPP

/**
 * ============================================================
 * Extended Kalman Filter (EKF) 
 * ESP32-S3 WROOM-1
 *
 * Vetor de estado (7D):
 *   x[0] = X      — posição local Este  [m]
 *   x[1] = Y      — posição local Norte [m]
 *   x[2] = Vx     — velocidade Este     [m/s]
 *   x[3] = Vy     — velocidade Norte    [m/s]
 *   x[4] = ax     — aceleração Este     [m/s²]
 *   x[5] = ay     — aceleração Norte    [m/s²]
 *   x[6] = psi    — yaw                 [rad]
 *
 * Entradas (já tratadas externamente):
 *   - speed_hall   : velocidade escalar do sensor Hall     [m/s]
 *   - accel_x_body : aceleração longitudinal do ICM       [m/s²]  (já em m/s², não em g)
 *   - euler_psi    : yaw confiável do ICM                 [rad]
 *   - gps_lat/lon  : coordenadas brutas do GPS            [°]
 * ============================================================
 */

#include <Arduino.h>
#include <math.h>

#define EKF_N 7          // dimensão do estado

class EKF {
public:

    // ── Estado e covariância ──────────────────────────────
    float x[EKF_N];               // vetor de estado estimado
    float P[EKF_N * EKF_N];       // covariância do erro

    // ── Ruídos ───────────────────────────────────────────
    float Q[EKF_N * EKF_N];       // processo
    float R_hall[4];              // medição Hall  (2×2, armazenado linha a linha)
    float R_icm[4];               // medição ICM   (2×2)
    float R_gps[4];               // medição GPS   (2×2)

    // ── Referência GPS ────────────────────────────────────
    bool   gps_ref_set;
    double ref_lat, ref_lon;      // [°]

    // ── Temporização ─────────────────────────────────────
    unsigned long last_time_us;

    // ── Resultado para leitura externa ───────────────────
    struct Estado {
        float X, Y;        // posição  [m]
        float Vx, Vy;      // velocidade [m/s]
        float speed;       // |V|      [m/s]
        float ax, ay;      // aceleração [m/s²]
        float accel;       // |a|      [m/s²]
        float psi;         // yaw      [rad]
    };

    // ── API pública ───────────────────────────────────────
    EKF();

    /**
     * Inicializa (ou reinicializa) o filtro.
     * Pode ser chamado de novo se quiser redefinir a referência GPS.
     */
    void init();

    /**
     * Ciclo completo: predição + atualização Hall + ICM.
     * Chame a cada iteração do loop principal (~50–100 Hz).
     *
     * @param speed_hall    Velocidade do sensor Hall [m/s]
     * @param accel_x_body  Aceleração longitudinal do ICM [m/s²]
     * @param euler_psi     Yaw do ICM [rad]
     */
    void atualiza(float speed_hall, float accel_x_body, float euler_psi);

    /**
     * Atualização com GPS — chame apenas quando chegar nova trama válida.
     *
     * @param lat  Latitude  [°]
     * @param lon  Longitude [°]
     */
    void atualizaGPS(double lat, double lon);

    /** Retorna o estado atual em uma estrutura legível. */
    Estado getEstado() const;

    /** Debug: imprime estado completo via Serial. */
    void printSerial() const;

private:
    // ── Álgebra linear compacta (sem alocação dinâmica) ──
    static void mat_copy(const float *A, float *B, int n);
    static void mat_transpose(const float *A, float *B, int r, int c);
    static void mat_mul(const float *A, const float *B, float *C, int m, int k, int n);
    static void mat_add(const float *A, const float *B, float *C, int n);
    static void mat_sub(const float *A, const float *B, float *C, int n);
    static bool mat_inv2(const float *A, float *inv);   // inversão 2×2 analítica
    static bool mat_inv7(const float *A, float *inv);   // inversão 7×7 Gauss-Jordan

    // ── Passos internos do EKF ────────────────────────────
    void predict(float dt);
    void updateHall(float speed_hall, float psi);
    void updateICM(float accel_x_body, float euler_psi);
    void updateGeneric2(const float z[2], const float h[2],
                        const float H[2 * EKF_N], const float R[4]);

    // ── GPS: conversão geodésica → ENU ───────────────────
    void gpsToLocal(double lat, double lon, float &X, float &Y) const;
};

#endif // EKF_HPP
