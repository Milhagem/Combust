#include "EKF.hpp"


void EKF::mat_copy(const float *A, float *B, int n) {
    for (int i = 0; i < n; i++) B[i] = A[i];
}

void EKF::mat_transpose(const float *A, float *B, int r, int c) {
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            B[j * r + i] = A[i * c + j];
}

void EKF::mat_mul(const float *A, const float *B, float *C, int m, int k, int n) {
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++) {
            float s = 0.0f;
            for (int l = 0; l < k; l++) s += A[i * k + l] * B[l * n + j];
            C[i * n + j] = s;
        }
}

void EKF::mat_add(const float *A, const float *B, float *C, int n) {
    for (int i = 0; i < n; i++) C[i] = A[i] + B[i];
}

void EKF::mat_sub(const float *A, const float *B, float *C, int n) {
    for (int i = 0; i < n; i++) C[i] = A[i] - B[i];
}

/** Inversão analítica de matriz 2×2 (armazenada linha a linha, tamanho 4). */
bool EKF::mat_inv2(const float *A, float *inv) {
    float det = A[0] * A[3] - A[1] * A[2];
    if (fabsf(det) < 1e-9f) return false;
    float inv_det = 1.0f / det;
    inv[0] =  A[3] * inv_det;
    inv[1] = -A[1] * inv_det;
    inv[2] = -A[2] * inv_det;
    inv[3] =  A[0] * inv_det;
    return true;
}

/** Inversão 7×7 por Gauss-Jordan — sem alocação dinâmica. */
bool EKF::mat_inv7(const float *A, float *inv) {
    const int n = EKF_N;
    float aug[EKF_N][2 * EKF_N];

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)     aug[i][j]     = A[i * n + j];
        for (int j = 0; j < n; j++)     aug[i][n + j] = (i == j) ? 1.0f : 0.0f;
    }

    for (int col = 0; col < n; col++) {
        int pivot = col;
        for (int row = col + 1; row < n; row++)
            if (fabsf(aug[row][col]) > fabsf(aug[pivot][col])) pivot = row;
        if (fabsf(aug[pivot][col]) < 1e-9f) return false;

        for (int j = 0; j < 2 * n; j++) {
            float t = aug[col][j]; aug[col][j] = aug[pivot][j]; aug[pivot][j] = t;
        }
        float d = aug[col][col];
        for (int j = 0; j < 2 * n; j++) aug[col][j] /= d;

        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            float f = aug[row][col];
            for (int j = 0; j < 2 * n; j++) aug[row][j] -= f * aug[col][j];
        }
    }

    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) inv[i * n + j] = aug[i][n + j];
    return true;
}

// ─────────────────────────────────────────────
//  Construtor e inicialização
// ─────────────────────────────────────────────

EKF::EKF() {
    init();
}

void EKF::init() {
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));
    memset(Q, 0, sizeof(Q));

    // ── Covariância inicial P ──
    // Incerteza inicial razoável para um carro parado num ponto desconhecido
    const float P0[EKF_N] = {
        25.0f,   // X      [m²]       ±10 m
        25.0f,   // Y      [m²]
        20.0f,     // Vx     [m²/s²]    ±2 m/s
        20.0f,     // Vy     [m²/s²]
        2.5f,     // ax     [m²/s⁴]    ±1 m/s²
        2.5f,     // ay     [m²/s⁴]
        0.01f     // psi    [rad²]      ±0.1 rad
    };
    for (int i = 0; i < EKF_N; i++) P[i * EKF_N + i] = P0[i];

    // ── Ruído de processo Q ──
    // Modela incertezas do modelo cinemático (jerk, slip, vento lateral…)
    // Aumente Q[ax]/Q[ay] se o carro tiver acelerações bruscas não modeladas.
    const float Qd[EKF_N] = {
        0.0005f,   // X
        0.0005f,   // Y
        1.05f,    // Vx
        1.05f,    // Vy
        1.50f,    // ax   — maior: jerk em oval pode ser alto
        1.50f,    // ay
        0.0005f   // psi  — yaw injetado diretamente, este Q quase não importa
    };
    for (int i = 0; i < EKF_N; i++) Q[i * EKF_N + i] = Qd[i];

    // ── Ruídos de medição R (matrizes 2×2 diagonais) ──

    // Hall: sensor de efeito Hall é preciso em velocidade escalar
    R_hall[0] = 0.04f;  R_hall[1] = 0.0f;   // σ²_Vx
    R_hall[2] = 0.0f;   R_hall[3] = 0.04f;  // σ²_Vy

    // ICM: aceleração já tratada — bastante confiável
    R_icm[0] = 0.5f;   R_icm[1] = 0.0f;    // σ²_ax
    R_icm[2] = 0.0f;    R_icm[3] = 0.5f;   // σ²_ay

    // GPS: ~2–5 m de erro típico em ambiente externo
    R_gps[0] = 1.0f;    R_gps[1] = 0.0f;    // σ²_X  [m²]
    R_gps[2] = 0.0f;    R_gps[3] = 1.0f;    // σ²_Y  [m²]

    gps_ref_set   = false;
    ref_lat       = 0.0;
    ref_lon       = 0.0;
    last_time_us  = micros();
}

// ─────────────────────────────────────────────
//  Atualização principal (Hall + ICM)
// ─────────────────────────────────────────────

void EKF::atualiza(float speed_hall, float accel_x_body, float euler_psi) {
    unsigned long now = micros();
    float dt = (now - last_time_us) * 1e-6f;
    last_time_us = now;

    // Proteção contra dt inválido (boot, overflow de micros)
    if (dt <= 0.0f || dt > 0.5f) return;

    predict(dt);
    updateHall(speed_hall, euler_psi);
    updateICM(accel_x_body, euler_psi);
}

// ─────────────────────────────────────────────
//  Predição
// ─────────────────────────────────────────────

void EKF::predict(float dt) {
    const float dt2_2 = 0.5f * dt * dt;

    // ── Propagação do estado ──
    // Modelo de aceleração constante no intervalo dt
    float xn[EKF_N];
    xn[0] = x[0] + x[2] * dt + x[4] * dt2_2;  // X
    xn[1] = x[1] + x[3] * dt + x[5] * dt2_2;  // Y
    xn[2] = x[2] + x[4] * dt;                  // Vx
    xn[3] = x[3] + x[5] * dt;                  // Vy
    xn[4] = x[4];                               // ax (constante)
    xn[5] = x[5];                               // ay (constante)
    xn[6] = x[6];                               // psi (injetado pelo ICM)
    mat_copy(xn, x, EKF_N);

    // ── Jacobiano F (7×7) ── identidade + termos de dt
    float F[EKF_N * EKF_N] = {};
    for (int i = 0; i < EKF_N; i++) F[i * EKF_N + i] = 1.0f;

    F[0 * EKF_N + 2] = dt;       // ∂X/∂Vx
    F[0 * EKF_N + 4] = dt2_2;    // ∂X/∂ax
    F[1 * EKF_N + 3] = dt;       // ∂Y/∂Vy
    F[1 * EKF_N + 5] = dt2_2;    // ∂Y/∂ay
    F[2 * EKF_N + 4] = dt;       // ∂Vx/∂ax
    F[3 * EKF_N + 5] = dt;       // ∂Vy/∂ay

    // ── P = F * P * F^T + Q ──
    float FP[EKF_N * EKF_N], FT[EKF_N * EKF_N], FPFT[EKF_N * EKF_N], Pnew[EKF_N * EKF_N];
    mat_mul(F, P, FP, EKF_N, EKF_N, EKF_N);
    mat_transpose(F, FT, EKF_N, EKF_N);
    mat_mul(FP, FT, FPFT, EKF_N, EKF_N, EKF_N);
    mat_add(FPFT, Q, Pnew, EKF_N * EKF_N);
    mat_copy(Pnew, P, EKF_N * EKF_N);
}

// ─────────────────────────────────────────────
//  Atualização genérica com observação 2D
// ─────────────────────────────────────────────

void EKF::updateGeneric2(const float z[2], const float h[2],
                          const float H[2 * EKF_N], const float R[4]) {
    const int n = EKF_N;

    // ── Inovação ──
    float y[2] = { z[0] - h[0], z[1] - h[1] };

    // ── S = H*P*H^T + R  (2×2) ──
    float HT[n * 2], HP[2 * n], HPHT[4], S[4], S_inv[4];
    mat_transpose(H, HT, 2, n);
    mat_mul(H, P, HP, 2, n, n);
    mat_mul(HP, HT, HPHT, 2, n, 2);
    mat_add(HPHT, R, S, 4);

    if (!mat_inv2(S, S_inv)) return;   // S singular → ignora medição

    // ── K = P * H^T * S^-1  (7×2) ──
    float PHT[n * 2], K[n * 2];
    mat_mul(P, HT, PHT, n, n, 2);
    mat_mul(PHT, S_inv, K, n, 2, 2);

    // ── x = x + K * y ──
    for (int i = 0; i < n; i++)
        x[i] += K[i * 2] * y[0] + K[i * 2 + 1] * y[1];

    // ── P = (I - K*H) * P ──
    float KH[n * n], I_KH[n * n], Pnew[n * n];
    mat_mul(K, H, KH, n, 2, n);
    memset(I_KH, 0, sizeof(I_KH));
    for (int i = 0; i < n; i++) I_KH[i * n + i] = 1.0f;
    mat_sub(I_KH, KH, I_KH, n * n);
    mat_mul(I_KH, P, Pnew, n, n, n);
    mat_copy(Pnew, P, n * n);
}

// ─────────────────────────────────────────────
//  Atualização — Sensor Hall
// ─────────────────────────────────────────────
/**
 * O Hall mede a velocidade escalar (|V|).
 * Projetamos usando o yaw para obter Vx e Vy observados.
 *
 * Observação:   z  = [cos(psi)*speed,  sin(psi)*speed]
 * Predição:     h  = [Vx,              Vy            ]
 * Jacobiano:    H[Vx] = 1,  H[Vy] = 1   (restante zero)
 *
 * Nota: a dependência de psi aparece em z (medição), não em h.
 * Isso é correto — z é o dado do sensor transformado para
 * world-frame usando o yaw já confiável do ICM.
 */
void EKF::updateHall(float speed_hall, float psi) {
    float cp = cosf(psi), sp = sinf(psi);

    float z[2] = { cp * speed_hall,  sp * speed_hall };
    float h[2] = { x[2],             x[3]            };

    float H[2 * EKF_N] = {};
    H[0 * EKF_N + 2] = 1.0f;   // ∂Vx_pred / ∂Vx
    H[1 * EKF_N + 3] = 1.0f;   // ∂Vy_pred / ∂Vy

    updateGeneric2(z, h, H, R_hall);
}

// ─────────────────────────────────────────────
//  Atualização — ICM (aceleração + yaw)
// ─────────────────────────────────────────────
/**
 * accel_x_body: aceleração frontal já em m/s² (body-frame, eixo X do carro).
 * Convertemos para world-frame usando o yaw.
 * Roll/pitch ≈ 0 em pista plana → apenas rotação em torno de Z é necessária.
 *
 * O yaw é injetado diretamente no estado (dado confiável, não integrado),
 * evitando deriva de integração.
 */
void EKF::updateICM(float accel_x_body, float euler_psi) {
    // Injeta yaw diretamente — dado confiável do ICM
    x[6] = euler_psi;

    float cp = cosf(euler_psi), sp = sinf(euler_psi);

    float z[2] = { accel_x_body * cp,  accel_x_body * sp };
    float h[2] = { x[4],               x[5]               };

    float H[2 * EKF_N] = {};
    H[0 * EKF_N + 4] = 1.0f;   // ∂ax_pred / ∂ax
    H[1 * EKF_N + 5] = 1.0f;   // ∂ay_pred / ∂ay

    updateGeneric2(z, h, H, R_icm);
}

// ─────────────────────────────────────────────
//  Atualização — GPS
// ─────────────────────────────────────────────

void EKF::gpsToLocal(double lat, double lon, float &X, float &Y) const {
    const double R_earth = 6371000.0;
    double dlat    = (lat - ref_lat) * M_PI / 180.0;
    double dlon    = (lon - ref_lon) * M_PI / 180.0;
    double lat_mid = (lat + ref_lat) * 0.5 * M_PI / 180.0;
    Y = (float)(R_earth * dlat);
    X = (float)(R_earth * dlon * cos(lat_mid));
}

void EKF::atualizaGPS(double lat, double lon) {
    if (!gps_ref_set) {
        ref_lat     = lat;
        ref_lon     = lon;
        gps_ref_set = true;
        x[0] = 0.0f;
        x[1] = 0.0f;
        Serial.println("[EKF] Referência GPS definida.");
        return;
    }

    float gX, gY;
    gpsToLocal(lat, lon, gX, gY);

    float z[2] = { gX, gY };
    float h[2] = { x[0], x[1] };

    float H[2 * EKF_N] = {};
    H[0 * EKF_N + 0] = 1.0f;   // ∂X_pred / ∂X
    H[1 * EKF_N + 1] = 1.0f;   // ∂Y_pred / ∂Y

    updateGeneric2(z, h, H, R_gps);
}

// ─────────────────────────────────────────────
//  Leitura de estado
// ─────────────────────────────────────────────

EKF::Estado EKF::getEstado() const {
    Estado e;
    e.X     = x[0];
    e.Y     = x[1];
    e.Vx    = x[2];
    e.Vy    = x[3];
    e.ax    = x[4];
    e.ay    = x[5];
    e.psi   = x[6];
    e.speed = sqrtf(e.Vx * e.Vx + e.Vy * e.Vy);
    e.accel = sqrtf(e.ax * e.ax + e.ay * e.ay);
    return e;
}

void EKF::printSerial() const {
    Estado e = getEstado();
    Serial.printf(
        "[EKF] X=%7.2f m  Y=%7.2f m | "
        "V=%5.2f m/s (Vx=%5.2f Vy=%5.2f) | "
        "a=%5.2f m/s2 (ax=%5.2f ay=%5.2f) | "
        "psi=%6.3f rad\n",
        e.X, e.Y,
        e.speed, e.Vx, e.Vy,
        e.accel, e.ax, e.ay,
        e.psi
    );
}
