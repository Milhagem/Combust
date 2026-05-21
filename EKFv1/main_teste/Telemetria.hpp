#ifndef TELEMETRIA_HPP
#define TELEMETRIA_HPP

#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ICM20948_WE.h>
#include <TinyGPS++.h>

#define ICM_ADDR 0x69
#define DEG2RAD  0.01745329251f
#define RAD2DEG  57.2957795131f

class KalmanIMU {
public:
    float x;    // estado estimado
    float P;    // covariância do erro
    float Q;    // ruído do processo
    float R;    // ruído da medição

    KalmanIMU(float Q, float R, float P = 1.0f, float x = 0.0f) 
        : Q(Q), R(R), P(P), x(x) {}

    void update(float measurement) {
        P += Q;
        float K = P / (P + R);
        x += K * (measurement - x);
        P *= (1 - K);
    }
};

class Telemetria {
public:

    Telemetria();
    void inicializaICM();
    void atualizaSensores();
    void inicializaGPS();
    void atualizaGPS();

    // ── Getters IMU ───────────────────────────────
    float getRoll()              const { return roll; }
    float getPitch()             const { return pitch; }
    float getYaw()               const { return yaw; }
    float getAccelLongitudinal() const { return accelLongitudinal; }

    // ── Getters GPS ───────────────────────────────
    float getLatitude()  const { return currentLat; }
    float getLongitude() const { return currentLon; }
    float getAltitude()  const { return currentAltitude; }
    float getSpeed()     const { return currentSpeed; }

    // WiFi/MQTT — reservado para uso futuro
    void EnviodadosWifi();

private:

    // ── ICM20948 ───────────────────────────────────
    ICM20948_WE imu;
    float roll, pitch, yaw, accelLongitudinal;
    KalmanIMU kalRoll, kalPitch, kalYaw;
    unsigned long lastMicros;

    static constexpr float magOffset[3] = {10.923861f, -9.444138f, 10.615106f};
    static constexpr float magSoftIron[3][3] = {
        {0.999398f, -0.020340f, 0.044319f},
        {-0.020340f, 0.946910f, 0.003804f},
        {0.044319f, 0.003804f, 1.059153f}
    };

    void applyMagCalibration(float &mx, float &my, float &mz);
    void processaKalman(float ax, float ay, float az,
                        float gx, float gy, float gz,
                        float mx, float my, float mz, float dt);
    void removeGravidade(float ax, float ay, float az,
                        float roll, float pitch,
                        float &ax_real, float &ay_real, float &az_real);

    // ── GPS ────────────────────────────────────────
    static constexpr int GPS_RX_PIN = 17;
    static constexpr int GPS_TX_PIN = 16;

    HardwareSerial gpsSerial;
    TinyGPSPlus gps;
    float currentLat      = 0.0f;
    float currentLon      = 0.0f;
    float currentAltitude = 0.0f;
    float currentSpeed    = 0.0f;

    // ── WiFi / MQTT ───────────────────────────────────────
    char payload[512];
    const char* ssid;
    const char* password;
    const char* mqtt_server;

    WiFiClient   espClient;
    PubSubClient client;
    unsigned long lastMsg;

    void setup_wifi();
    void reconnect();

};  // ← SEMICOLON

#endif // TELEMETRIA_HPP