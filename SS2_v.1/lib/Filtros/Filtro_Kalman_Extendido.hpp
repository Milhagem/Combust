#ifndef FILTRO_KALMAN_EXTENDIDO_HPP
#define FILTRO_KALMAN_EXTENDIDO_HPP

#include <Arduino.h>
#include <math.h>
#include <string.h>

#define EKF_N 5

class FiltroKalmanExtendido {
public:
    struct Estado {
        float X;
        float Y;
        float v;
        float theta;
        float omega_bias;
        float ax;
    };

    struct DadosGPS {
        bool valido;
        double latitude;
        double longitude;
        float x_m;
        float y_m;
    };

    FiltroKalmanExtendido();

    void init();

    void atualizarIMU(float omega_z, float ax, float speed_hall);
    void atualizarGPS(double lat, double lon);

    Estado getEstado() const;
    DadosGPS getUltimaPosicaoGPS() const;
    void printSerial() const;

    void setQ(int idx, float val);
    void setR(const char *sensor, float val);

private:
    float x[EKF_N];
    float P[EKF_N * EKF_N];
    float Q_diag[EKF_N];

    float R_mag;
    float R_gps_pos;
    float R_hall;

    bool gps_ref_set;
    double ref_lat;
    double ref_lon;
    unsigned long last_us;

    bool gps_valid;
    double last_latitude;
    double last_longitude;
    float last_gps_x_m;
    float last_gps_y_m;
    float last_ax;

    void prever(float ax, float omega_z, float dt);
    void atualizarHeading(float theta_mag);
    void atualizarVelocidadeRoda(float speed_hall);
    void atualizarPosicaoGPS(float gX, float gY);
    void updateScalar(const float H[EKF_N], float innov, float R);
    void gpsToLocal(double lat, double lon, float &X, float &Y) const;
    static float wrapAngle(float a);
};

using VehicleEKF = FiltroKalmanExtendido;

#endif
