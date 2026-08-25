#include "Filtro_Kalman_Extendido.hpp"

float FiltroKalmanExtendido::wrapAngle(float a) {
    while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

FiltroKalmanExtendido::FiltroKalmanExtendido()
    : gps_ref_set(false), ref_lat(0.0), ref_lon(0.0), last_us(0), last_ax(0.0f) {
    init();
}

void FiltroKalmanExtendido::init() {
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));

    const float P0[EKF_N] = {
        15.0f,
        15.0f,
        1.0f,
        0.5f,
        0.01f
    };
    for (int i = 0; i < EKF_N; i++) P[i * EKF_N + i] = P0[i];

    const float Qd[EKF_N] = {
        0.02f,
        0.02f,
        0.20f,
        0.01f,
        0.0005f
    };
    memcpy(Q_diag, Qd, sizeof(Qd));

    // Removido R_mag
    R_gps_pos = 4.0f;
    R_hall = 0.04f;

    gps_ref_set = false;
    ref_lat = 0.0;
    ref_lon = 0.0;
    gps_valid = false;
    last_latitude = 0.0;
    last_longitude = 0.0;
    last_gps_x_m = 0.0f;
    last_gps_y_m = 0.0f;
    last_us = micros();
    last_ax = 0.0f;
}

void FiltroKalmanExtendido::setQ(int idx, float val) {
    if (idx >= 0 && idx < EKF_N) Q_diag[idx] = val;
}

void FiltroKalmanExtendido::setR(const char *sensor, float val) {
    if (strcmp(sensor, "gps_pos") == 0) {
        R_gps_pos = val;
    } else if (strcmp(sensor, "hall") == 0) {
        R_hall = val;
    }
}

// =========================================================
// NOVOS GETTERS PARA O CALLBACK MQTT
// =========================================================
float FiltroKalmanExtendido::getQ(int idx) const {
    if (idx >= 0 && idx < EKF_N) {
        return Q_diag[idx];
    }
    return -1.0f; // Valor inválido
}

float FiltroKalmanExtendido::getR(const char *sensor) const {
    if (strcmp(sensor, "gps_pos") == 0) {
        return R_gps_pos;
    } else if (strcmp(sensor, "hall") == 0) {
        return R_hall;
    }
    return -1.0f; // Valor inválido
}

void FiltroKalmanExtendido::atualizarIMU(float omega_z, float ax, float speed_hall) {
    unsigned long now = micros();
    float dt = (now - last_us) * 1e-6f;
    last_us = now;

    if (dt <= 0.0f || dt > 0.2f) return;

    last_ax = ax;

    prever(ax, omega_z, dt);
    atualizarVelocidadeRoda(speed_hall);
}

void FiltroKalmanExtendido::atualizarGPS(double lat, double lon) {
    if (!isfinite(lat) || !isfinite(lon)) {
        gps_valid = false;
        return;
    }

    if (!gps_ref_set) {
        ref_lat = lat;
        ref_lon = lon;
        gps_ref_set = true;
        x[0] = 0.0f;
        x[1] = 0.0f;
        gps_valid = true;
        last_latitude = lat;
        last_longitude = lon;
        last_gps_x_m = 0.0f;
        last_gps_y_m = 0.0f;
        Serial.println("[FiltroKalmanExtendido] Referência GPS definida.");
        return;
    }

    float gX, gY;
    gpsToLocal(lat, lon, gX, gY);
    gps_valid = true;
    last_latitude = lat;
    last_longitude = lon;
    last_gps_x_m = gX;
    last_gps_y_m = gY;
    atualizarPosicaoGPS(gX, gY);
}

void FiltroKalmanExtendido::prever(float ax, float omega_z, float dt) {
    float px = x[0], py = x[1], v = x[2], th = x[3], wb = x[4];
    float c = cosf(th);
    float s = sinf(th);
    float w_corr = omega_z - wb;

    x[0] = px + v * c * dt;
    x[1] = py + v * s * dt;
    x[2] = v + ax * dt;
    x[3] = wrapAngle(th + w_corr * dt);
    x[4] = wb;

    float F[EKF_N * EKF_N];
    memset(F, 0, sizeof(F));
    for (int i = 0; i < EKF_N; i++) F[i * EKF_N + i] = 1.0f;

    F[0 * EKF_N + 2] = c * dt;
    F[0 * EKF_N + 3] = -v * s * dt;
    F[1 * EKF_N + 2] = s * dt;
    F[1 * EKF_N + 3] = v * c * dt;
    F[3 * EKF_N + 4] = -dt;

    static float FP[EKF_N * EKF_N];
    static float FPFt[EKF_N * EKF_N];

    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float s_ = 0.0f;
            for (int k = 0; k < EKF_N; k++) s_ += F[i * EKF_N + k] * P[k * EKF_N + j];
            FP[i * EKF_N + j] = s_;
        }
    }

    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float s_ = 0.0f;
            for (int k = 0; k < EKF_N; k++) s_ += FP[i * EKF_N + k] * F[j * EKF_N + k];
            FPFt[i * EKF_N + j] = s_;
        }
    }

    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            P[i * EKF_N + j] = FPFt[i * EKF_N + j];
        }
    }

    for (int i = 0; i < EKF_N; i++) {
        P[i * EKF_N + i] += Q_diag[i] * dt;
    }
}

void FiltroKalmanExtendido::updateScalar(const float H[EKF_N], float innov, float R) {
    const int N = EKF_N;

    float PHt[N];
    for (int i = 0; i < N; i++) {
        float s = 0.0f;
        for (int j = 0; j < N; j++) s += P[i * N + j] * H[j];
        PHt[i] = s;
    }

    float S = R;
    for (int j = 0; j < N; j++) S += H[j] * PHt[j];

    if (fabsf(S) < 1e-9f) return;

    float S_inv = 1.0f / S;

    float K[N];
    for (int i = 0; i < N; i++) K[i] = PHt[i] * S_inv;

    for (int i = 0; i < N; i++) x[i] += K[i] * innov;
    x[3] = wrapAngle(x[3]);

    float HP[N];
    for (int j = 0; j < N; j++) {
        float s = 0.0f;
        for (int k = 0; k < N; k++) s += H[k] * P[k * N + j];
        HP[j] = s;
    }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            P[i * N + j] -= K[i] * HP[j];
        }
    }
}

// Removida a função atualizarHeading() que utilizava theta_mag

void FiltroKalmanExtendido::atualizarVelocidadeRoda(float speed_hall) {
    if (speed_hall < 0.0f) return;

    float H[EKF_N] = {0, 0, 1, 0, 0};

    float R = R_hall;
    if (speed_hall < 0.3f) R *= 25.0f;

    float innov = speed_hall - x[2];
    updateScalar(H, innov, R);
}

void FiltroKalmanExtendido::atualizarPosicaoGPS(float gX, float gY) {
    float Hx[EKF_N] = {1, 0, 0, 0, 0};
    float Hy[EKF_N] = {0, 1, 0, 0, 0};

    updateScalar(Hx, gX - x[0], R_gps_pos);
    updateScalar(Hy, gY - x[1], R_gps_pos);
}

void FiltroKalmanExtendido::gpsToLocal(double lat, double lon, float &X, float &Y) const {
    const double R_earth = 6371000.0;
    double dlat = (lat - ref_lat) * M_PI / 180.0;
    double dlon = (lon - ref_lon) * M_PI / 180.0;
    double lat_mid = (lat + ref_lat) * 0.5 * M_PI / 180.0;
    Y = (float)(R_earth * dlat);
    X = (float)(R_earth * dlon * cos(lat_mid));
}

FiltroKalmanExtendido::Estado FiltroKalmanExtendido::getEstado() const {
    Estado e;
    e.X = x[0];
    e.Y = x[1];
    e.v = x[2];
    e.theta = x[3];
    e.omega_bias = x[4];
    e.ax = last_ax;
    return e;
}

FiltroKalmanExtendido::DadosGPS FiltroKalmanExtendido::getUltimaPosicaoGPS() const {
    DadosGPS gps;
    gps.valido = gps_valid;
    gps.latitude = last_latitude;
    gps.longitude = last_longitude;
    gps.x_m = last_gps_x_m;
    gps.y_m = last_gps_y_m;
    return gps;
}

void FiltroKalmanExtendido::printSerial() const {
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