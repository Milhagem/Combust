#include "EKF.hpp"

// ══════════════════════════════════════════════════════════════
//  Álgebra linear compacta (sem alocação dinâmica)
// ══════════════════════════════════════════════════════════════

void AckermanEKF::mat_zero(float *A, int n) {
    memset(A, 0, sizeof(float) * n);
}

void AckermanEKF::mat_eye(float *A, int n) {
    mat_zero(A, n * n);
    for (int i = 0; i < n; i++) A[i * n + i] = 1.0f;
}

void AckermanEKF::mat_copy(const float *src, float *dst, int n) {
    memcpy(dst, src, sizeof(float) * n);
}

void AckermanEKF::mat_add(const float *A, const float *B, float *C, int n) {
    for (int i = 0; i < n; i++) C[i] = A[i] + B[i];
}

void AckermanEKF::mat_sub(const float *A, const float *B, float *C, int n) {
    for (int i = 0; i < n; i++) C[i] = A[i] - B[i];
}

// C[m×n] = A[m×k] * B[k×n]
void AckermanEKF::mat_mul(const float *A, const float *B, float *C,
                           int m, int k, int n) {
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++) {
            float s = 0.0f;
            for (int l = 0; l < k; l++) s += A[i * k + l] * B[l * n + j];
            C[i * n + j] = s;
        }
}

// C[m×n] = A[m×k] * B[n×k]^T   (evita alocar B transposta)
void AckermanEKF::mat_mulT(const float *A, const float *B, float *C,
                            int m, int k, int n) {
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++) {
            float s = 0.0f;
            for (int l = 0; l < k; l++) s += A[i * k + l] * B[j * k + l];
            C[i * n + j] = s;
        }
}

float AckermanEKF::wrapAngle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

// ══════════════════════════════════════════════════════════════
//  Construtor e inicialização
// ══════════════════════════════════════════════════════════════

AckermanEKF::AckermanEKF(float wheelbase_m, float Tc_s)
    : L(wheelbase_m), Tc(Tc_s), v_current(0.0f),
      gps_ref_set(false), ref_lat(0.0), ref_lon(0.0),
      last_us(0)
{
    init();
}

void AckermanEKF::init() {
    mat_zero(x, AEKF_N);
    mat_zero(P, AEKF_N * AEKF_N);

    // ── Covariância inicial P (diagonal) ──────────────────────
    // Estado            σ inicial
    // X, Y              ±5 m     → P = 25
    // theta             ±0.5 rad → P = 0.25
    // theta_B (bias)    ±0.3 rad → P = 0.09
    // x_B, y_B (bias GPS) ±3 m  → P = 9
    // x_hp, x_lp (filtro) ±0.3 rad → P = 0.09
    // delta             ±0.5 rad (~28°) → P = 0.25
    const float P0[AEKF_N] = {
        25.0f,   // X
        25.0f,   // Y
        0.25f,   // theta
        0.09f,   // theta_B
        9.0f,    // x_B
        9.0f,    // y_B
        0.09f,   // x_hp
        0.09f,   // x_lp
        0.25f    // delta
    };
    for (int i = 0; i < AEKF_N; i++) P[i * AEKF_N + i] = P0[i];

    // ── Ruído de processo Q (diagonal, por segundo) ───────────
    // Mais alto = o EKF confia menos no modelo e mais nas medições.
    // theta_B, x_B, y_B: deriva lenta → Q pequeno.
    // delta: pode mudar moderadamente → Q médio.
    const float Qd[AEKF_N] = {
        0.01f,    // X       [m²/s]
        0.01f,    // Y       [m²/s]
        0.001f,   // theta   [rad²/s]
        0.0001f,  // theta_B [rad²/s]  — deriva muito lenta
        0.005f,   // x_B     [m²/s]
        0.005f,   // y_B     [m²/s]
        0.001f,   // x_hp    [rad²/s]
        0.001f,   // x_lp    [rad²/s]
        0.005f    // delta   [rad²/s]  — permite curvas moderadas
                  //                    aumente para manobras mais ágeis
    };
    mat_copy(Qd, Q_diag, AEKF_N);

    // ── Ruídos de medição R ───────────────────────────────────
    R_imu         = 0.04f;    // ≈ 0.2 rad de desvio padrão (~11°) — ICM20948 dinâmico
    R_gps_pos     = 4.0f;     // ≈ 2 m de desvio padrão — GPS de baixo custo
    R_gps_bearing = 0.09f;    // ≈ 0.3 rad (~17°)
    R_gyro        = 0.001f;   // giroscópio Z: sensor preciso ≈ 0.032 rad/s

    gps_ref_set = false;
    ref_lat     = 0.0;
    ref_lon     = 0.0;
    last_us     = micros();
    v_current   = 0.0f;
}

// ══════════════════════════════════════════════════════════════
//  Interface pública de sintonização
// ══════════════════════════════════════════════════════════════

void AckermanEKF::setQ(int idx, float val) {
    if (idx >= 0 && idx < AEKF_N) Q_diag[idx] = val;
}

void AckermanEKF::setR(const char *sensor, float val) {
    if      (strcmp(sensor, "imu_bearing")  == 0) R_imu         = val;
    else if (strcmp(sensor, "gps_pos")      == 0) R_gps_pos     = val;
    else if (strcmp(sensor, "gps_bearing")  == 0) R_gps_bearing = val;
    else if (strcmp(sensor, "gyro_z")       == 0) R_gyro        = val;
}

// ══════════════════════════════════════════════════════════════
//  Ciclo IMU (~75 Hz)
// ══════════════════════════════════════════════════════════════

void AckermanEKF::atualizaIMU(float theta_imu, float omega_z, float speed_mps) {
    unsigned long now = micros();
    float dt = (now - last_us) * 1e-6f;
    last_us  = now;

    if (dt <= 0.0f || dt > 0.2f) return;  // protege contra overflow ou boot

    v_current = speed_mps;

    // ── O filtro complementar precisa de theta_GPS para x_lp ──
    // Usamos o último valor de x_lp como aproximação do theta_GPS filtrado
    // (ele só muda quando chega GPS)
    float theta_gps_lp = x[7];   // estado interno da componente passa-baixa

    predict(dt, theta_imu, theta_gps_lp);

    // ── Atualização com IMU bearing ──────────────────────────
    updateIMUbearing(theta_imu);

    // ── Atualização com bearing fusionado (filtro complementar) ─
    float theta_fused_meas = x[6] + x[7];   // y_hp + y_lp  (equação 4 do artigo)
    updateFusedBearing(theta_fused_meas);

    // ── Atualização com giroscópio Z (estima delta) ──────────
    // Só válido quando em movimento (evita instabilidade numérica perto de v=0)
    if (fabsf(v_current) > AEKF_MIN_SPEED_GPS) {
        updateGyroZ(omega_z);
    }
}

// ══════════════════════════════════════════════════════════════
//  Atualização GPS (~5 Hz)
// ══════════════════════════════════════════════════════════════

void AckermanEKF::atualizaGPS(double lat, double lon, float theta_gps) {
    // ── Primeira trama: define referência ENU ────────────────
    if (!gps_ref_set) {
        ref_lat     = lat;
        ref_lon     = lon;
        gps_ref_set = true;
        x[0] = 0.0f;
        x[1] = 0.0f;
        Serial.println("[AckermanEKF] Referência GPS definida.");
        return;
    }

    float gX, gY;
    gpsToLocal(lat, lon, gX, gY);

    // ── Posição GPS (com compensação de bias) ─────────────────
    updateGPSpos(gX, gY);

    // ── Bearing GPS (só se válido e em movimento) ─────────────
    if (!isnanf(theta_gps) && fabsf(v_current) > AEKF_MIN_SPEED_GPS) {
        updateGPSbearing(theta_gps);

        // Atualiza componente passa-baixa do filtro complementar
        // x_lp converge para theta_GPS com constante de tempo Tc
        // Usamos o dt médio do GPS (0.2 s @ 5 Hz)
        const float dt_gps = 0.2f;
        float alpha_lp = dt_gps / (Tc + dt_gps);
        x[7] = (1.0f - alpha_lp) * x[7] + alpha_lp * theta_gps;
    }
}

// ══════════════════════════════════════════════════════════════
//  Predição — modelo Ackerman + bias + filtro complementar
// ══════════════════════════════════════════════════════════════

void AckermanEKF::predict(float dt, float theta_imu, float theta_gps_lp) {
    const float N = AEKF_N;
    float th  = x[2];   // heading atual
    float del = x[8];   // delta atual

    // ── Propagação não-linear do estado (equações 1a-1c + 2 + 3) ──
    float xn[AEKF_N];

    // Posição: equações 1a, 1b
    xn[0] = x[0] + v_current * cosf((float)M_PI_2 - th) * dt;   // X
    xn[1] = x[1] + v_current * sinf((float)M_PI_2 - th) * dt;   // Y

    // Heading: equação 1c
    float theta_dot = (L > 0.01f) ? (v_current / L) * tanf(del) : 0.0f;
    xn[2] = wrapAngle(th + theta_dot * dt);

    // Biases: passeio aleatório (permanecem constantes na predição)
    xn[3] = x[3];   // theta_B
    xn[4] = x[4];   // x_B
    xn[5] = x[5];   // y_B

    // Filtro complementar — equação (3) do artigo (discretizado):
    // x_hp[k+1] = x_hp[k] * (1 - dt/Tc) + theta_imu * dt/Tc
    // x_lp[k+1] = x_lp[k] * (1 - dt/Tc) + theta_gps_lp * dt/Tc
    float alpha = dt / Tc;
    if (alpha > 1.0f) alpha = 1.0f;
    xn[6] = x[6] * (1.0f - alpha) + theta_imu  * alpha;   // x_hp
    xn[7] = x[7] * (1.0f - alpha) + theta_gps_lp * alpha; // x_lp

    // Delta: passeio aleatório
    xn[8] = x[8];   // delta

    mat_copy(xn, x, AEKF_N);

    // ── Jacobiano A (linearização em torno do estado atual) ──
    // Apenas os elementos não-nulos são escritos; o restante é zero.
    // Baseado na equação (9) do artigo, expandida para 9 estados.
    float A[AEKF_N * AEKF_N];
    mat_zero(A, AEKF_N * AEKF_N);

    // ∂Xdot/∂theta = v * sin(pi/2 - theta)
    A[0 * AEKF_N + 2] =  v_current * sinf((float)M_PI_2 - th);

    // ∂Ydot/∂theta = -v * cos(pi/2 - theta)
    A[1 * AEKF_N + 2] = -v_current * cosf((float)M_PI_2 - th);

    // ∂thetadot/∂delta = (v/L) / cos²(delta)   — jacobiano NOVO (Abordagem 2)
    if (L > 0.01f) {
        float cos_d = cosf(del);
        if (fabsf(cos_d) > 0.05f) {  // evita divisão por zero perto de ±90°
            A[2 * AEKF_N + 8] = (v_current / L) / (cos_d * cos_d);
        }
    }

    // Filtro complementar: A[6][6] = A[7][7] = -1/Tc  (equação 9 do artigo)
    A[6 * AEKF_N + 6] = -1.0f / Tc;
    A[7 * AEKF_N + 7] = -1.0f / Tc;

    // ── Phi ≈ I + A*dt  (aproximação de 1ª ordem — adequada para dt < 15 ms) ──
    float Phi[AEKF_N * AEKF_N];
    mat_eye(Phi, AEKF_N);
    for (int i = 0; i < AEKF_N * AEKF_N; i++) Phi[i] += A[i] * dt;

    // ── P = Phi * P * Phi^T + Q*dt ────────────────────────────
    float PhiP  [AEKF_N * AEKF_N];
    float PhiPPhiT[AEKF_N * AEKF_N];

    mat_mul (Phi, P,    PhiP,      AEKF_N, AEKF_N, AEKF_N);
    mat_mulT(PhiP, Phi, PhiPPhiT,  AEKF_N, AEKF_N, AEKF_N);

    // Adiciona Q*dt na diagonal
    for (int i = 0; i < AEKF_N; i++)
        PhiPPhiT[i * AEKF_N + i] += Q_diag[i] * dt;

    mat_copy(PhiPPhiT, P, AEKF_N * AEKF_N);
}

// ══════════════════════════════════════════════════════════════
//  Núcleo genérico — atualização com observações independentes
//  Processa cada linha de H separadamente (S escalar → sem inversão)
// ══════════════════════════════════════════════════════════════

void AckermanEKF::updateCore(const float *z, const float *h,
                              const float *H, const float *R_diag, int nz) {
    const int N = AEKF_N;

    for (int obs = 0; obs < nz; obs++) {
        const float *Hi = H + obs * N;  // linha obs de H

        // ── S = Hi * P * Hi^T + R_i  (escalar) ───────────────
        float PHiT[N];   // P * Hi^T
        for (int i = 0; i < N; i++) {
            float s = 0.0f;
            for (int j = 0; j < N; j++) s += P[i * N + j] * Hi[j];
            PHiT[i] = s;
        }
        float S = 0.0f;
        for (int j = 0; j < N; j++) S += Hi[j] * PHiT[j];
        S += R_diag[obs];

        if (fabsf(S) < 1e-9f) continue;   // degenerado → ignora
        float S_inv = 1.0f / S;

        // ── K = PHiT * S_inv  (vetor N×1) ────────────────────
        float K[N];
        for (int i = 0; i < N; i++) K[i] = PHiT[i] * S_inv;

        // ── Inovação ─────────────────────────────────────────
        float innov = z[obs] - h[obs];

        // Normaliza inovação angular se o estado for um ângulo
        // (estados 2, 3, 6, 7 são ângulos)
        // Aplica wrapping apenas quando a inovação parece angular
        if (fabsf(innov) > (float)M_PI * 0.5f) {
            innov = wrapAngle(innov);
        }

        // ── x = x + K * innov ────────────────────────────────
        for (int i = 0; i < N; i++) x[i] += K[i] * innov;
        x[2] = wrapAngle(x[2]);   // mantém theta em [-pi, pi]
        x[8] = wrapAngle(x[8]);   // mantém delta em [-pi, pi]

        // ── P = (I - K*Hi) * P ───────────────────────────────
        // Implementado como P -= K * (Hi * P) para evitar buffer N×N extra
        float HiP[N];   // Hi * P  (vetor linha)
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) s += Hi[k] * P[k * N + j];
            HiP[j] = s;
        }
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                P[i * N + j] -= K[i] * HiP[j];
    }
}

// ══════════════════════════════════════════════════════════════
//  Atualizações individuais
// ══════════════════════════════════════════════════════════════

// ── Bearing do IMU (magnetômetro + bias) ─────────────────────
// Observação: z = theta_IMU = theta + theta_B  (equação Hk linha 1)
void AckermanEKF::updateIMUbearing(float theta_imu) {
    float H[AEKF_N]; mat_zero(H, AEKF_N);
    H[2] = 1.0f;   // ∂z/∂theta
    H[3] = 1.0f;   // ∂z/∂theta_B

    float h = x[2] + x[3];
    float z = theta_imu;
    float R = R_imu;

    updateCore(&z, &h, H, &R, 1);
}

// ── Bearing fusionado (filtro complementar) ──────────────────
// Observação: z = theta_fused = theta + theta_B - (1/Tc)*x_hp + x_lp
// (equação Hk linha 2, segunda linha da equação 11 do artigo)
void AckermanEKF::updateFusedBearing(float theta_fused_meas) {
    float H[AEKF_N]; mat_zero(H, AEKF_N);
    H[2] =  1.0f;         // ∂z/∂theta
    H[3] =  1.0f;         // ∂z/∂theta_B
    H[6] = -1.0f / Tc;   // ∂z/∂x_hp
    H[7] =  1.0f;         // ∂z/∂x_lp

    float h = x[2] + x[3] + (-1.0f / Tc) * x[6] + x[7];
    float z = theta_fused_meas;
    float R = R_imu * 0.5f;  // fusionado é mais confiável que IMU puro

    updateCore(&z, &h, H, &R, 1);
}

// ── Posição GPS (com compensação de bias) ────────────────────
// Observação: z = [x_GPS, y_GPS] = [X + x_B, Y + y_B]
// (equações Hk linhas 4-5 da equação 11 do artigo)
void AckermanEKF::updateGPSpos(float gX, float gY) {
    float H[2 * AEKF_N]; mat_zero(H, 2 * AEKF_N);
    H[0 * AEKF_N + 0] = 1.0f;   // ∂z_x/∂X
    H[0 * AEKF_N + 4] = 1.0f;   // ∂z_x/∂x_B
    H[1 * AEKF_N + 1] = 1.0f;   // ∂z_y/∂Y
    H[1 * AEKF_N + 5] = 1.0f;   // ∂z_y/∂y_B

    float z[2] = { gX,               gY               };
    float h[2] = { x[0] + x[4],     x[1] + x[5]      };
    float R[2] = { R_gps_pos,        R_gps_pos        };

    updateCore(z, h, H, R, 2);
}

// ── Bearing GPS ───────────────────────────────────────────────
// Observação: z = theta_GPS = theta + theta_B  (mesma observação do IMU,
// mas com R diferente — GPS tem ruído menor em baixas freq.)
void AckermanEKF::updateGPSbearing(float theta_gps) {
    float H[AEKF_N]; mat_zero(H, AEKF_N);
    H[2] = 1.0f;   // ∂z/∂theta
    H[3] = 1.0f;   // ∂z/∂theta_B

    float h = x[2] + x[3];
    float z = theta_gps;
    float R = R_gps_bearing;

    updateCore(&z, &h, H, &R, 1);
}

// ── Giroscópio Z (estima delta) — Abordagem 3 ────────────────
// Modelo: omega_z ≈ (v/L) * tan(delta)
// Jacobiano: ∂omega_z/∂delta = (v/L) / cos²(delta)
void AckermanEKF::updateGyroZ(float omega_z) {
    float H[AEKF_N]; mat_zero(H, AEKF_N);

    float del   = x[8];
    float cos_d = cosf(del);
    if (fabsf(cos_d) < 0.05f) return;   // delta > ~87° → ignora

    float dh_ddelta = (fabsf(L) > 0.01f)
                      ? (v_current / L) / (cos_d * cos_d)
                      : 0.0f;
    H[8] = dh_ddelta;   // ∂omega_z/∂delta

    float h = (L > 0.01f) ? (v_current / L) * tanf(del) : 0.0f;
    float z = omega_z;
    float R = R_gyro;

    updateCore(&z, &h, H, &R, 1);
}

// ══════════════════════════════════════════════════════════════
//  Conversão GPS → ENU local
// ══════════════════════════════════════════════════════════════

void AckermanEKF::gpsToLocal(double lat, double lon, float &X, float &Y) const {
    const double R_earth = 6371000.0;
    double dlat     = (lat - ref_lat) * M_PI / 180.0;
    double dlon     = (lon - ref_lon) * M_PI / 180.0;
    double lat_mid  = (lat + ref_lat) * 0.5 * M_PI / 180.0;
    Y = (float)(R_earth * dlat);
    X = (float)(R_earth * dlon * cos(lat_mid));
}

// ══════════════════════════════════════════════════════════════
//  Leitura e debug
// ══════════════════════════════════════════════════════════════

AckermanEKF::Estado AckermanEKF::getEstado() const {
    Estado e;
    e.X            = x[0];
    e.Y            = x[1];
    e.theta        = x[2];
    e.delta        = x[8];
    e.theta_fused  = x[6] + x[7];   // bearing fusionado: y_hp + y_lp
    e.speed        = v_current;
    return e;
}

void AckermanEKF::printSerial() const {
    Estado e = getEstado();
    Serial.printf(
        "[EKF] X=%7.2f m  Y=%7.2f m | "
        "theta=%6.3f rad (%6.1f°) | "
        "delta=%5.3f rad (%5.1f°) | "
        "v=%5.2f m/s | "
        "fused=%6.3f rad\n",
        e.X, e.Y,
        e.theta, e.theta * 57.2957f,
        e.delta, e.delta * 57.2957f,
        e.speed,
        e.theta_fused
    );
}
