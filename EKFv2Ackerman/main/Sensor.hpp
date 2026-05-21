#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <ICM20948_WE.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include "Kalman.hpp"
// Nota: AckermanEKF usa Sensor como fonte de dados, nao o contrario.
// Nao inclua EKF.hpp aqui para evitar dependencia circular.

#define ICM_ADDR 0x69
#define DEG2RAD  0.01745329251f
#define RAD2DEG  57.2957795131f

#define N_IMAS       2.0
#define CIRCUNF_RODA 1.81
#define G_ACEL       9.80665
#define MS2_TO_KMHS  3.6

class Sensor {
private:

    // PARTE DO ICM
    ICM20948_WE imu;
    float roll, pitch, yaw, accelLongitudinal;
    Kalman kalRoll, kalPitch, kalYaw;
    unsigned long lastMicros;

    //CALIBRAÇÃO MAGNETÔMETRO (veja verbete no notion)
    static constexpr float magOffset[3] = {10.923861f, -9.444138f, 10.615106f};
    static constexpr float magSoftIron[3][3] = {
        {0.999398f, -0.020340f, 0.044319f},
        {-0.020340f, 0.946910f, 0.003804f},
        {0.044319f, 0.003804f, 1.059153f}
    };

    //CALIBRAÇÃO GIROSCÓPIO
    struct GyroOffset {
        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;
    };

    GyroOffset gyroOffset;
    bool calibrando = false;

    // Valores calculados em atualizaSensores() e exportados para AckermanEKF
    float gyroZ_filtered = 0.0f;   // gz com bias removido e filtro passa-baixo [rad/s]
    float magBearing     = 0.0f;   // bearing magnético calibrado + declinacao   [rad]

    // Declinacao magnetica local (configure para sua regiao)
    // Belo Horizonte, MG: aprox. -21.7 graus W -> valor negativo
    // Calcule em: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
    static constexpr float MAG_DECLINATION = -21.7f * DEG2RAD;

    // FILTRO PASSA-BAIXO (reduz vibracoes)
    struct LowPassFilter {
        float alpha = 0.15f;  // Fator de suavização (menor = mais suavização)
        float last_value = 0.0f;
        
        float apply(float raw_value) {
            last_value = alpha * raw_value + (1.0f - alpha) * last_value;
            return last_value;
        }
    };

    LowPassFilter lpAccX, lpAccY, lpAccZ;   // Acelerômetro
    LowPassFilter lpGyrX, lpGyrY, lpGyrZ;   // Giroscópio
    LowPassFilter lpMagX, lpMagY, lpMagZ;   // Magnetômetro

    void applyMagCalibration(float &mx, float &my, float &mz);
    void processaKalman(float ax, float ay, float az,
                        float gx, float gy, float gz,
                        float mx, float my, float mz, float dt);
    void removeGravidade(float ax, float ay, float az,
                        float roll, float pitch,
                        float &ax_real, float &ay_real, float &az_real);

    // PARTE DO GPS
    static constexpr int GPS_RX_PIN = 17;
    static constexpr int GPS_TX_PIN = 16;

    HardwareSerial gpsSerial;
    TinyGPSPlus gps;
    float currentLat      = 0.0f;
    float currentLon      = 0.0f;
    float currentAltitude = 0.0f;
    float currentSpeed    = 0.0f;

    // PARTE DO HALL
    uint8_t pino;
    bool    hallHabilitado;

    volatile unsigned long periodo;
    volatile unsigned long ultimoTempo;
    volatile unsigned long ultimoDebounce;
    volatile double        rpmAtual;
    volatile double        rpmAntigo;

public:

    Sensor(uint8_t pino_sensor, bool habilitarHall = true);
    void inicializaICM();
    void atualizaSensores();
    void inicializaGPS();
    void atualizaGPS();

    void calibrarGiroscopio(uint16_t amostras = 500);

    // ── Getters IMU (público) ─────────────────────────────
    float getRoll()              const { return roll; }
    float getPitch()             const { return pitch; }
    float getYaw()               const { return yaw; }
    float getAccelLongitudinal() const { return accelLongitudinal; }

    /**
     * Giroscópio Z com bias já removido, em rad/s.
     * Usado pelo AckermanEKF para estimar o ângulo de esterço (delta).
     */
    float getGyroZ() const { return gyroZ_filtered; }

    /**
     * Bearing magnético calibrado (soft-iron + hard-iron + declinação).
     * Retorna atan2(my_cal, mx_cal) + declinação magnética local.
     * Usado pelo AckermanEKF como theta_IMU.
     */
    float getMagBearing() const { return magBearing; }

    // ── Getters GPS (público) ─────────────────────────────
    float getLatitude()  const { return currentLat; }
    float getLongitude() const { return currentLon; }
    float getAltitude()  const { return currentAltitude; }
    float getSpeed()     const { return currentSpeed; }

    // PARTE DO HALL
    void setHallHabilitado(bool habilitado);
    bool isHallHabilitado() const;
    void calc();
    double getVelocidadeHALL();
    double getVelocidadeHALL_kmh();
    static float gParaMs2(float accel_g);

};

#endif // SENSOR_HPP