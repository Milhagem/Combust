#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Endereços e Registradores do ASM330LHHX
#define ASM330LHHX_ADDR 0x6B
#define WHO_AM_I_REG    0x0F
#define CTRL1_XL        0x10 
#define CTRL2_G         0x11 
#define OUTX_L_G        0x22 
#define OUTX_L_A        0x28 

#define DEG2RAD 0.01745329251f
#define RAD2DEG 57.2957795131f

class IMU {
private:
    TwoWire* _wire;
    uint8_t  _address;
    
    // Pinos I2C
    int _sdaPin;
    int _sclPin;

    // Tempos
    unsigned long lastMicros;

    // Dados brutos e processados
    float roll, pitch, yaw;
    float accelLongitudinal;
    float gyroZ_filtered;

    // Offset do Giroscópio
    struct GyroOffset {
        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;
    } gyroOffset;

    // Filtros Passa-Baixo (para reduzir vibração do motor nas leituras brutas)
    struct LowPassFilter {
        float alpha = 0.15f; 
        float last_value = 0.0f;
        float apply(float raw_value) {
            last_value = alpha * raw_value + (1.0f - alpha) * last_value;
            return last_value;
        }
    };

    LowPassFilter lpAccX, lpAccY, lpAccZ;
    LowPassFilter lpGyrX, lpGyrY, lpGyrZ;

    // Funções internas I2C
    void writeRegister(uint8_t reg, uint8_t value);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);

    // Funções matemáticas internas
    void removeGravidade(float ax, float ay, float az, float roll_rad, float pitch_rad, float &ax_real, float &ay_real, float &az_real);

public:
    // Construtor
    IMU(int sdaPin = 8, int sclPin = 9, TwoWire* wire = &Wire, uint8_t address = ASM330LHHX_ADDR);

    // Inicialização e Calibração
    bool begin(uint32_t frequency = 400000);
    void calibrarGiroscopio(uint16_t amostras = 500);

    // Leitura e Processamento (chamar no loop)
    void update();
    void readRawData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

    // Getters
    float getRoll()              const { return roll; }              // [rad]
    float getPitch()             const { return pitch; }             // [rad]
    float getYaw()               const { return yaw; }               // [rad]
    float getAccelLongitudinal() const { return accelLongitudinal; } // [m/s^2]
    float getGyroZ()             const { return gyroZ_filtered; }    // [rad/s]
};

#endif // IMU_HPP