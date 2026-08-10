#include "Filtro_Kalman_Extendido.hpp"

// =================================================================
// INICIALIZAÇÃO
// =================================================================
void Filtro_Kalman_Extendido::begin() {
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));

    // Covariância Inicial P (Alta incerteza inicial)
    P[0 * EKF_N + 0] = 15.0f;  // X
    P[1 * EKF_N + 1] = 15.0f;  // Y
    P[2 * EKF_N + 2] = 1.0f;   // v
    P[3 * EKF_N + 3] = 0.5f;   // theta
    P[4 * EKF_N + 4] = 0.01f;  // omega_bias

    // Ruído de Processo Q (Incerteza do modelo)
    Q_diag[0] = 0.02f;   // X
    Q_diag[1] = 0.02f;   // Y
    Q_diag[2] = 0.20f;   // v
    Q_diag[3] = 0.01f;   // theta
    Q_diag[4] = 0.0005f; // omega_bias

    gps_ref_set = false;
    last_us = micros();
}

// =================================================================
// SINTONIA EM TEMPO DE EXECUÇÃO (Q e R)
// =================================================================

void Filtro_Kalman_Extendido::setQ(int idx, float val) {
    // Verifica se o índice é válido (0 a 4) para evitar invasão de memória
    if (idx >= 0 && idx < EKF_N) {
        Q_diag[idx] = val;
    }
}

void Filtro_Kalman_Extendido::setR(const char* sensor, float val) {
    // Compara a string recebida (ex: do MQTT) e atualiza o ruído correspondente
    if (strcmp(sensor, "gps_pos") == 0) {
        R_gps_pos = val;
    } 
    else if (strcmp(sensor, "hall") == 0) {
        R_hall = val;
    }
    else if (strcmp(sensor, "gps_heading") == 0) {
        R_gps_heading = val;
    }
}

// =================================================================
// START-STOP: CONGELAMENTO DE ESTADO
// =================================================================
void Filtro_Kalman_Extendido::setVeiculoParado(bool parado) {
    veiculo_parado = parado;
    if (parado) {
        x[2] = 0.0f; // Força velocidade a zero
    }
}

// =================================================================
// PREDIÇÃO (Modelo Bicicleta Cinemático)
// =================================================================
void Filtro_Kalman_Extendido::predict(float ax, float omega_z, float dt) {
    // Se o sensor Hall diz que está parado, a física congela.
    if (veiculo_parado) {
        ax = 0.0f;
        x[2] = 0.0f;
        // Não somamos Q_diag nas posições X, Y e V para evitar que 
        // a incerteza do EKF "exploda" enquanto o carro está parado no semáforo.
    }

    last_ax = ax;

    float px = x[0], py = x[1], v = x[2], th = x[3], wb = x[4];
    float c = cosf(th);
    float s = sinf(th);
    float w_corr = omega_z - wb;

    // Atualização de Estado Não-Linear
    x[0] = px + v * c * dt;
    x[1] = py + v * s * dt;
    x[2] = v + ax * dt;
    x[3] = wrapAngle(th + w_corr * dt);
    x[4] = wb;

    // Jacobiano F e Covariância P (Lógica padrão EKF mantida)
    float F[EKF_N * EKF_N] = {0};
    for (int i = 0; i < EKF_N; i++) F[i * EKF_N + i] = 1.0f;
    F[0 * EKF_N + 2] = c * dt;
    F[0 * EKF_N + 3] = -v * s * dt;
    F[1 * EKF_N + 2] = s * dt;
    F[1 * EKF_N + 3] = v * c * dt;
    F[3 * EKF_N + 4] = -dt;

    float FP[EKF_N * EKF_N] = {0};
    float FPFt[EKF_N * EKF_N] = {0};

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            for (int k = 0; k < EKF_N; k++) 
                FP[i * EKF_N + j] += F[i * EKF_N + k] * P[k * EKF_N + j];

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            for (int k = 0; k < EKF_N; k++) 
                FPFt[i * EKF_N + j] += FP[i * EKF_N + k] * F[j * EKF_N + k];

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            P[i * EKF_N + j] = FPFt[i * EKF_N + j];

    // Só injeta ruído de processo (Q) se estiver em movimento (Start-Stop Logic)
    if (!veiculo_parado) {
        for (int i = 0; i < EKF_N; i++) P[i * EKF_N + i] += Q_diag[i] * dt;
    } else {
        // Se parado, permite apenas o drift do bias do giroscópio crescer
        P[4 * EKF_N + 4] += Q_diag[4] * dt; 
    }
}

// =================================================================
// ATUALIZAÇÃO SENSOR HALL
// =================================================================
void Filtro_Kalman_Extendido::updateHall(float speed_hall) {
    if (speed_hall < 0.0f) return;
    float H[EKF_N] = {0, 0, 1, 0, 0};
    
    // Dinâmica de ruído: em baixa velocidade, desconfia mais do sensor
    float R = R_hall;
    if (speed_hall < 0.3f) R *= 25.0f;
    
    updateScalar(H, speed_hall - x[2], R);
}

// =================================================================
// ATUALIZAÇÃO GPS (Posição e Heading Dinâmico)
// =================================================================
void Filtro_Kalman_Extendido::updateGPS(double lat, double lon, float gps_speed, float gps_course) {
    if (!gps_ref_set) {
        ref_lat = lat;
        ref_lon = lon;
        gps_ref_set = true;
        x[0] = 0.0f; x[1] = 0.0f;
        return;
    }

    float gX, gY;
    gpsToLocal(lat, lon, gX, gY);

    float Hx[EKF_N] = {1, 0, 0, 0, 0};
    float Hy[EKF_N] = {0, 1, 0, 0, 0};

    updateScalar(Hx, gX - x[0], R_gps_pos);
    updateScalar(Hy, gY - x[1], R_gps_pos);
}

// =================================================================
// UTILITÁRIOS MATEMÁTICOS
// =================================================================
float Filtro_Kalman_Extendido::wrapAngle(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

void Filtro_Kalman_Extendido::gpsToLocal(double lat, double lon, float &X, float &Y) {
    const double R_earth = 6371000.0;
    double dlat = (lat - ref_lat) * M_PI / 180.0;
    double dlon = (lon - ref_lon) * M_PI / 180.0;
    double lat_mid = (lat + ref_lat) * 0.5 * M_PI / 180.0;
    Y = (float)(R_earth * dlat);
    X = (float)(R_earth * dlon * cos(lat_mid));
}

void Filtro_Kalman_Extendido::updateScalar(const float H[EKF_N], float innov, float R) {
    float PHt[EKF_N] = {0};
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++) PHt[i] += P[i * EKF_N + j] * H[j];

    float S = R;
    for (int j = 0; j < EKF_N; j++) S += H[j] * PHt[j];

    if (fabsf(S) < 1e-9f) return;
    float S_inv = 1.0f / S;

    float K[EKF_N];
    for (int i = 0; i < EKF_N; i++) K[i] = PHt[i] * S_inv;

    for (int i = 0; i < EKF_N; i++) x[i] += K[i] * innov;
    x[3] = wrapAngle(x[3]);

    float HP[EKF_N] = {0};
    for (int j = 0; j < EKF_N; j++)
        for (int k = 0; k < EKF_N; k++) HP[j] += H[k] * P[k * EKF_N + j];

    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            P[i * EKF_N + j] -= K[i] * HP[j];
}