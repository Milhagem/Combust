#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <math.h>
#include <Wire.h>

// =================================================================
// CONSTANTES E REGISTRADORES
// =================================================================
#define ASM330LHHX_ADDR 0x6B
#define WHO_AM_I_REG    0x0F
#define CTRL1_XL        0x10 
#define CTRL2_G         0x11 
#define OUTX_L_G        0x22 
#define OUTX_L_A        0x28 

#define DEG2RAD 0.01745329251f
#define RAD2DEG 57.2957795131f

// =================================================================
// STRUCTS AUXILIARES (Definidas fora da classe para evitar erros do compilador)
// =================================================================
struct GyroOffset {
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;
};

struct LowPassFilter {
    float alpha = 0.15f; 
    float last_value = 0.0f;
    float apply(float raw_value) {
        last_value = alpha * raw_value + (1.0f - alpha) * last_value;
        return last_value;
    }
};

// =================================================================
// CLASSE IMU (Totalmente Estática)
// =================================================================
class IMU {
private:
    inline static TwoWire* _wire = &Wire;
    inline static uint8_t  _address = ASM330LHHX_ADDR;
    
    // Pinos I2C
    inline static int _sdaPin = 8;
    inline static int _sclPin = 9;

    // Tempos
    inline static unsigned long lastMicros = 0;

    // Dados brutos e processados
    inline static float roll = 0.0f;
    inline static float pitch = 0.0f;
    inline static float yaw = 0.0f;
    inline static float accelLongitudinal = 0.0f;
    inline static float accelCentrifuga = 0.0f;
    inline static float gyroZ_filtered = 0.0f;

    // Instâncias das structs agora funcionam perfeitamente
    inline static GyroOffset gyroOffset;
    inline static LowPassFilter lpAccX, lpAccY, lpAccZ;
    inline static LowPassFilter lpGyrX, lpGyrY, lpGyrZ;

    // Funções internas I2C
    static void writeRegister(uint8_t reg, uint8_t value);
    static void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);

    // Funções matemáticas internas
    static void removeGravidade(float ax, float ay, float az, float roll_rad, float pitch_rad, float &ax_real, float &ay_real, float &az_real);

public:
    // Configuração Opcional (Substitui o Construtor)
    static void setPins(int sda, int scl);

    // Inicialização e Calibração
    static bool begin(uint32_t frequency = 400000);
    static void calibrarGiroscopio(uint16_t amostras = 500);

    // Leitura e Processamento (chamar no loop)
    static void update();
    static void readRawData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

    // Getters
    static float getRoll();
    static float getPitch();
    static float getYaw();
    static float getAccelLongitudinal();
    static float getAccelCentrifuga();
    static float getGyroZ();
};

#endif