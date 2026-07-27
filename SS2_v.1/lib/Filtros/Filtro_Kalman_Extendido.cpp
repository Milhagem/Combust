#include "Filtro_Kalman_Extendido.hpp"

// ══════════════════════════════════════════════════════════════
//  Útil
// ══════════════════════════════════════════════════════════════

float Filtro_Kalman_Extendido::wrapAngle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

// ══════════════════════════════════════════════════════════════
//  Construtor e inicialização
// ══════════════════════════════════════════════════════════════

Filtro_Kalman_Extendido::Filtro_Kalman_Extendido()
    : gps_ref_set(false), ref_lat(0.0), ref_lon(0.0),
      last_us(0), last_ax(0.0f)
{
    init();
}

void Filtro_Kalman_Extendido::init() {
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));

    // ── Covariância inicial P (diagonal) ──────────────────────
    const float P0[EKF_N] = {
        15.0f,   // X        [m²]
        15.0f,   // Y        [m²]
        1.0f,    // v        [(m/s)²]
        0.5f,    // theta    [rad²]  (~40°) — incerteza inicial alta
        0.01f    // omega_bias [(rad/s)²]
    };
    for (int i = 0; i < EKF_N; i++) P[i * EKF_N + i] = P0[i];

    // ── Ruído de processo Q (diagonal, por segundo) ───────────
    // Mais alto = o EKF confia menos no modelo e mais nas medições.
    const float Qd[EKF_N] = {
        0.02f,    // X       [m²/s]   — incerteza adicional de integração
        0.02f,    // Y       [m²/s]
        0.20f,    // v       [(m/s)²/s] — IMU ax pode ter ruído/offset residual
        0.01f,    // theta   [rad²/s]
        0.0005f   // omega_bias [(rad/s)²/s] — drift lento do giro
    };
    memcpy(Q_diag, Qd, sizeof(Qd));

    // ── Ruídos de medição R ───────────────────────────────────
    R_mag     = 0.05f;   // ≈ 0.22 rad (~13°) — magnetômetro do ICM20948
    R_gps_pos = 4.0f;    // ≈ 2 m de desvio padrão — GPS de baixo custo
    R_hall    = 0.04f;   // ≈ 0.2 m/s de desvio padrão

    gps_ref_set = false;
    ref_lat = 0.0;
    ref_lon = 0.0;
    last_us = micros();
    last_ax = 0.0f;
}

// ══════════════════════════════════════════════════════════════
//  Interface pública de sintonização
// ══════════════════════════════════════════════════════════════

void Filtro_Kalman_Extendido::setQ(int idx, float val) {
    if (idx >= 0 && idx < EKF_N) Q_diag[idx] = val;
}

void Filtro_Kalman_Extendido::setR(const char *sensor, float val) {
    if      (strcmp(sensor, "mag")     == 0) R_mag     = val;
    else if (strcmp(sensor, "gps_pos") == 0) R_gps_pos = val;
    else if (strcmp(sensor, "hall")    == 0) R_hall    = val;
}

// ══════════════════════════════════════════════════════════════
//  Ciclo IMU (~75 Hz) — predição + correções de heading e Hall
// ══════════════════════════════════════════════════════════════

void Filtro_Kalman_Extendido::atualizaIMU(float theta_mag, float omega_z, float ax, float speed_hall) {
    unsigned long now = micros();
    float dt = (now - last_us) * 1e-6f;
    last_us  = now;

    if (dt <= 0.0f || dt > 0.2f) return;  // protege contra overflow/boot

    last_ax = ax;

    predict(ax, omega_z, dt);

    // ── Correção de heading (magnetômetro) ────────────────────
    updateHeading(theta_mag);

    // ── Correção de velocidade (Hall) ──────────────────────────
    updateWheelSpeed(speed_hall);
}

// ══════════════════════════════════════════════════════════════
//  Atualização GPS (~5 Hz)
// ══════════════════════════════════════════════════════════════

void Filtro_Kalman_Extendido::atualizaGPS(double lat, double lon) {
    // ── Primeira trama: define referência ENU ────────────────
    if (!gps_ref_set) {
        ref_lat     = lat;
        ref_lon     = lon;
        gps_ref_set = true;
        x[0] = 0.0f;
        x[1] = 0.0f;
        Serial.println("[VehicleEKF] Referência GPS definida.");
        return;
    }

    float gX, gY;
    gpsToLocal(lat, lon, gX, gY);
    updateGPSpos(gX, gY);
}

// ══════════════════════════════════════════════════════════════
//  Predição — modelo bicicleta cinemático
// ══════════════════════════════════════════════════════════════

void Filtro_Kalman_Extendido::predict(float ax, float omega_z, float dt) {
    float px = x[0], py = x[1], v = x[2], th = x[3], wb = x[4];

    float c = cosf(th);
    float s = sinf(th);
    float w_corr = omega_z - wb;

    // ── modelo de processo (não-linear) ───────────────────────
    x[0] = px + v * c * dt;
    x[1] = py + v * s * dt;
    x[2] = v  + ax * dt;
    x[3] = wrapAngle(th + w_corr * dt);
    x[4] = wb; // bias assumido constante (random walk via Q)

    // ── Jacobiano F = I + (dF/dx)*dt ───────────────────────────
    float F[EKF_N * EKF_N];
    memset(F, 0, sizeof(F));
    for (int i = 0; i < EKF_N; i++) F[i * EKF_N + i] = 1.0f;

    F[0 * EKF_N + 2] =  c * dt;          // dX/dv
    F[0 * EKF_N + 3] = -v * s * dt;      // dX/dtheta
    F[1 * EKF_N + 2] =  s * dt;          // dY/dv
    F[1 * EKF_N + 3] =  v * c * dt;      // dY/dtheta
    F[3 * EKF_N + 4] = -dt;              // dtheta/domega_bias

    // ── P = F*P*F^T + Q*dt ──────────────────────────────────────
    static float FP[EKF_N * EKF_N];
    static float FPFt[EKF_N * EKF_N];

    // FP = F * P
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++) {
            float s_ = 0.0f;
            for (int k = 0; k < EKF_N; k++) s_ += F[i * EKF_N + k] * P[k * EKF_N + j];
            FP[i * EKF_N + j] = s_;
        }

    // FPFt = FP * F^T
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++) {
            float s_ = 0.0f;
            for (int k = 0; k < EKF_N; k++) s_ += FP[i * EKF_N + k] * F[j * EKF_N + k];
            FPFt[i * EKF_N + j] = s_;
        }

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            P[i * EKF_N + j] = FPFt[i * EKF_N + j];

    for (int i = 0; i < EKF_N; i++)
        P[i * EKF_N + i] += Q_diag[i] * dt;
}

// ══════════════════════════════════════════════════════════════
//  Núcleo genérico — atualização escalar (S escalar, sem inversão)
// ══════════════════════════════════════════════════════════════

void Filtro_Kalman_Extendido::updateScalar(const float H[EKF_N], float innov, float R) {
    const int N = EKF_N;

    // PHt = P * H^T
    float PHt[N];
    for (int i = 0; i < N; i++) {
        float s = 0.0f;
        for (int j = 0; j < N; j++) s += P[i * N + j] * H[j];
        PHt[i] = s;
    }

    // S = H * P * H^T + R
    float S = R;
    for (int j = 0; j < N; j++) S += H[j] * PHt[j];

    if (fabsf(S) < 1e-9f) return; // degenerado

    float S_inv = 1.0f / S;

    // K = PHt / S
    float K[N];
    for (int i = 0; i < N; i++) K[i] = PHt[i] * S_inv;

    // x = x + K * innov
    for (int i = 0; i < N; i++) x[i] += K[i] * innov;
    x[3] = wrapAngle(x[3]);

    // P = (I - K*H) * P   =>   P -= K * (H * P)
    float HP[N];
    for (int j = 0; j < N; j++) {
        float s = 0.0f;
        for (int k = 0; k < N; k++) s += H[k] * P[k * N + j];
        HP[j] = s;
    }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            P[i * N + j] -= K[i] * HP[j];
}

// ══════════════════════════════════════════════════════════════
//  Correções individuais
// ══════════════════════════════════════════════════════════════

// ── Heading do magnetômetro: z = theta_mag,  h(x) = theta ────
void Filtro_Kalman_Extendido::updateHeading(float theta_mag) {
    float H[EKF_N] = {0, 0, 0, 1, 0};
    float innov = wrapAngle(theta_mag - x[3]);
    updateScalar(H, innov, R_mag);
}

// ── Velocidade do sensor Hall: z = v_roda,  h(x) = v ──────────
void Filtro_Kalman_Extendido::updateWheelSpeed(float speed_hall) {
    if (speed_hall < 0.0f) return; // valor inválido

    float H[EKF_N] = {0, 0, 1, 0, 0};

    // Em baixa velocidade o sensor Hall fica ruidoso (quantização de período).
    // Aumenta R dinamicamente perto de v ~ 0 para não puxar 'v' para baixo
    // de forma instável.
    float R = R_hall;
    if (speed_hall < 0.3f) R *= 25.0f;

    float innov = speed_hall - x[2];
    updateScalar(H, innov, R);
}

// ── Posição GPS: z = [X_gps, Y_gps],  h(x) = [X, Y] ───────────
void Filtro_Kalman_Extendido::updateGPSpos(float gX, float gY) {
    float Hx[EKF_N] = {1, 0, 0, 0, 0};
    float Hy[EKF_N] = {0, 1, 0, 0, 0};

    updateScalar(Hx, gX - x[0], R_gps_pos);
    updateScalar(Hy, gY - x[1], R_gps_pos);
}

// ══════════════════════════════════════════════════════════════
//  Conversão GPS → ENU local
// ══════════════════════════════════════════════════════════════

void Filtro_Kalman_Extendido::gpsToLocal(double lat, double lon, float &X, float &Y) const {
    const double R_earth = 6371000.0;
    double dlat    = (lat - ref_lat) * M_PI / 180.0;
    double dlon    = (lon - ref_lon) * M_PI / 180.0;
    double lat_mid = (lat + ref_lat) * 0.5 * M_PI / 180.0;
    Y = (float)(R_earth * dlat);
    X = (float)(R_earth * dlon * cos(lat_mid));
}

// ══════════════════════════════════════════════════════════════
//  Leitura e debug
// ══════════════════════════════════════════════════════════════

Filtro_Kalman_Extendido::Estado Filtro_Kalman_Extendido::getEstado() const {
    Estado e;
    e.X           = x[0];
    e.Y           = x[1];
    e.v           = x[2];
    e.theta       = x[3];
    e.omega_bias  = x[4];
    e.ax          = last_ax;
    return e;
}

void Filtro_Kalman_Extendido::printSerial() const {
    Estado e = getEstado();
    Serial.printf(
        "[EKF] X=%7.2f m  Y=%7.2f m | "
        "v=%5.2f m/s | "
        "theta=%6.3f rad (%6.1f°) | "
        "omega_bias=%6.4f rad/s | "
        "ax=%5.2f m/s²\n",
        e.X, e.Y,
        e.v,
        e.theta, e.theta * 57.2957f,
        e.omega_bias,
        e.ax
    );
}
