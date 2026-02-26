#ifndef TELEMETRIA_HPP
#define TELEMETRIA_HPP


#include <ICM20948_WE.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>


#define ICM_ADDR 0x69
#define DEG2RAD 0.01745329251f
#define RAD2DEG 57.2957795131f

class Telemetria {
public:

    // ================= KALMAN PARA IMU ===================
    class KalmanIMU {
        public:
            float x;   // estado (ângulo)
            float P;   // covariância
            float Q;   // ruído do processo
            float R;   // ruído da medição

            KalmanIMU(float q, float r) {
                Q = q;
                R = r;
                x = 0.0f;
                P = 1.0f;
            }

            // Previsão (gyro)
            void predict(float rate, float dt) {
                x += rate * dt;
                P += Q;
            }

            // Correção (acc / mag)
            float update(float z) {
                float K = P / (P + R);  
                x += K * (z - x);
                P *= (1.0f - K);
                return x;
            }

            void setQ(float q) { Q = q; }
            void setR(float r) { R = r; }
    };

    Telemetria();
    void inicializaICM();
    void atualizaSensores();
    void EnviodadosWifi();

private:

    ICM20948_WE imu;

    float roll, pitch, yaw, accelLongitudinal;

    KalmanIMU kalRoll, kalPitch, kalYaw;
    unsigned long lastMicros;

    // ================= CALIBRAÇÃO MAGNETÔMETRO ===================
    // Veja verbete, sempre refaça a calibração e cole os valores aqui
    static constexpr float magOffset[3] = {9.089679f, -13.885231f, -4.271233f};

    static constexpr float magSoftIron[3][3] = {
        {1.000000f, 0.000000f, 0.000000f},
        {0.000000f, 1.000000f, 0.000000f},
        {0.000000f, 0.000000f, 1.000000f}
    };

    //================== PARTE DO IMU ===================
    
    void applyMagCalibration(float &mx, float &my, float &mz); 
    void processaKalman(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt);
    void removeGravidade(float ax, float ay, float az, float roll, float pitch, float &ax_real, float &ay_real, float &az_real);

    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }
    float getAccelLongitudinal() const { return accelLongitudinal; }

    //================== PARTE DO GPS ===================



    //================== PARTE COMUNICAÇÃO WIFI ===================
    char payload[512];
    const char* ssid;
    const char* password;
    const char* mqtt_server;

    WiFiClient espClient;
    PubSubClient client;
    unsigned long lastMsg;

    void setup_wifi();
    void reconnect();
};

#endif