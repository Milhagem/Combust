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

//----------------------------------------------------------Velocidade.hpp-------------------------------------------------------------
#ifndef VELOCIDADE_HPP
#define VELOCIDADE_HPP

#include <Arduino.h>

class Velocidade {
public:
    class Kalman {
        private:
            double x[2];        // Estado: [0]=velocidade, [1]=aceleração
            double P[2][2];     // Covariância do erro
            double Q[2][2];     // Ruído do processo
            double R_vel;       // Ruído da medição de velocidade
            double R_ace;       // Ruído da medição de aceleração

        public:
            /**
             * @param q_acc Incerteza da aceleração (Process Noise)
             * @param r_vel Incerteza do sensor de velocidade (Measurement Noise)
             */
            Kalman(double q_acc, double r_vel) {
                x[0] = 0.0; x[1] = 0.0;
                
                // Inicializa P como matriz Identidade
                P[0][0] = 1.0; P[0][1] = 0.0;
                P[1][0] = 0.0; P[1][1] = 1.0;

                // Inicializa Q (Modelo de ruído para sistemas dinâmicos)
                Q[0][0] = 0.01; Q[0][1] = 0.0;
                Q[1][0] = 0.0;  Q[1][1] = q_acc;

                R_vel = r_vel;
                R_ace = q_acc * 0.5;  // Ruído de aceleração (menor que de velocidade)
            }

            /**
             * Passo de Predição: Projeta o estado atual para o futuro
             * @param a_icm Aceleração medida do ICM em m/s²
             * @param dt Intervalo de tempo desde a última leitura
             */
            void predict(double a_icm, double dt) {
                // Usa aceleração do ICM + aceleração derivada do estado para predição
                // Combina a aceleração medida com a estimada
                double a_combinada = 0.7 * a_icm + 0.3 * x[1];
                
                // x = F * x com aceleração combinada
                x[0] = x[0] + (a_combinada * dt);
                x[1] = a_combinada;

                // P = F * P * F' + Q
                double p00 = P[0][0] + dt * (P[1][0] + P[0][1] + dt * P[1][1]) + Q[0][0];
                double p01 = P[0][1] + dt * P[1][1] + Q[0][1];
                double p10 = P[1][0] + dt * P[1][1] + Q[1][0];
                double p11 = P[1][1] + Q[1][1];
                
                P[0][0] = p00; P[0][1] = p01;
                P[1][0] = p10; P[1][1] = p11;
            }

            /**
             * Atualização com múltiplas medidas
             * Funde velocidade RPM + velocidade integrada + aceleração ICM
             * @param z_vel_rpm Velocidade do RPM em m/s
             * @param z_vel_integrada Velocidade integrada da aceleração em m/s
             * @param z_ace_icm Aceleração do ICM em m/s²
             */
            void updateComMultiplasMedidas(double z_vel_rpm, double z_vel_integrada, double z_ace_icm) {
                // Fusão das duas medidas de velocidade
                double z_vel = (z_vel_rpm + z_vel_integrada) * 0.5;
                
                // ===== ATUALIZAÇÃO DE VELOCIDADE =====
                double S_vel = P[0][0] + R_vel;
                double K_vel[2];
                K_vel[0] = P[0][0] / S_vel;
                K_vel[1] = P[1][0] / S_vel;

                double y_vel = z_vel - x[0];
                x[0] += K_vel[0] * y_vel;
                x[1] += K_vel[1] * y_vel;

                double p00 = (1.0 - K_vel[0]) * P[0][0];
                double p01 = (1.0 - K_vel[0]) * P[0][1];
                double p10 = P[1][0] - (K_vel[1] * P[0][0]);
                double p11 = P[1][1] - (K_vel[1] * P[0][1]);

                P[0][0] = p00; P[0][1] = p01;
                P[1][0] = p10; P[1][1] = p11;
                
                // ===== ATUALIZAÇÃO DE ACELERAÇÃO =====
                double S_ace = P[1][1] + R_ace;
                double K_ace = P[1][1] / S_ace;
                double y_ace = z_ace_icm - x[1];
                x[1] += K_ace * y_ace;
                P[1][1] = (1.0 - K_ace) * P[1][1];
            }

            double getVelocidade() const { return x[0]; }
            double getAceleracao() const { return x[1]; }
    };

private:
    uint8_t pino;

    volatile unsigned long periodo;
    volatile unsigned long ultimoTempo;
    volatile unsigned long ultimoDebounce;
    volatile double rpmAtual;
    volatile double rpmAntigo;

    double ultimaVelocidade;
    unsigned long ultimoTempoAcc;
    double velocidadeIntegrada;  // Velocidade integrada a partir da aceleração ICM

    Kalman kalman;

public:

    Velocidade(uint8_t pino_sensor);

    void calc();
    
    /**
     * Retorna velocidade em m/s a partir do sensor Hall
     * @return Velocidade do RPM em m/s
     */
    double getVelocidadeHALL();
    
    /**
     * Atualiza o filtro com múltiplas medidas
     * Funde: RPM + aceleração ICM + velocidade integrada
     * @param aceleracao_icm Aceleração longitudinal do ICM em g
     * @param dt Intervalo de tempo em segundos
     */
    void atualizaComMultiplasMedidas(double aceleracao_icm, double dt);

    /**
     * Retorna velocidade em km/h
     * @return Velocidade filtrada em km/h
     */
    double getVelocidadeKmh() const;
    
    /**
     * Retorna aceleração filtrada em m/s²
     * @return Aceleração estimada em m/s²
     */
    double getAceleracaoMs2() const;

};

#endif