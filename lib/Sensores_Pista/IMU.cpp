#include "IMU.hpp"

// ==========================================
// Implementação da Classe IMU
// ==========================================
IMU::IMU(int sdaPin, int sclPin, TwoWire* wire, uint8_t address)
    : _sdaPin(sdaPin), _sclPin(sclPin), _wire(wire), _address(address),
      lastMicros(0), roll(0), pitch(0), yaw(0),
      accelLongitudinal(0), accelCentrifuga(0), gyroZ_filtered(0) 
{
}

void IMU::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
}

void IMU::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, len);
    for (uint8_t i = 0; i < len; i++) {
        if (_wire->available()) buffer[i] = _wire->read();
    }
}

bool IMU::begin(uint32_t frequency) {
    _wire->begin(_sdaPin, _sclPin);
    _wire->setClock(frequency);
    
    _wire->beginTransmission(_address);
    _wire->write(WHO_AM_I_REG);
    if (_wire->endTransmission(false) != 0) return false;
    
    _wire->requestFrom(_address, (uint8_t)1);
    if (_wire->read() != 0x6B) return false;

    writeRegister(CTRL1_XL, 0x40); // 104 Hz, +-2g
    writeRegister(CTRL2_G, 0x40);  // 104 Hz, +-250 dps
    
    lastMicros = micros();
    return true;
}

void IMU::calibrarGiroscopio(uint16_t amostras) {
    Serial.println("[IMU] Iniciando calibração do giroscópio...");
    Serial.println("[IMU] MANTENHA O SENSOR COMPLETAMENTE PARADO!");
    
    gyroOffset.gx = 0.0f;
    gyroOffset.gy = 0.0f;
    gyroOffset.gz = 0.0f;
    
    for (uint16_t i = 0; i < amostras; i++) {
        float ax, ay, az, gx, gy, gz;
        readRawData(ax, ay, az, gx, gy, gz);
        
        gyroOffset.gx += gx;
        gyroOffset.gy += gy;
        gyroOffset.gz += gz;
        
        if (i % 100 == 0) Serial.print(".");
        delay(10);
    }
    
    gyroOffset.gx /= amostras;
    gyroOffset.gy /= amostras;
    gyroOffset.gz /= amostras;
    
    Serial.println("\n[IMU] Calibração concluída!");
    Serial.printf("[IMU] Offsets (dps) X: %.2f, Y: %.2f, Z: %.2f\n", 
                  gyroOffset.gx, gyroOffset.gy, gyroOffset.gz);
}

void IMU::readRawData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    uint8_t buffer[6];
    
    // Leitura Acelerômetro
    readRegisters(OUTX_L_A, buffer, 6);
    int16_t rawAX = (buffer[1] << 8) | buffer[0];
    int16_t rawAY = (buffer[3] << 8) | buffer[2];
    int16_t rawAZ = (buffer[5] << 8) | buffer[4];
    
    float sensAcel = 0.000061f * 9.81f; // Converte +-2g direto para m/s^2
    ax = rawAX * sensAcel;
    ay = rawAY * sensAcel;
    az = rawAZ * sensAcel;

    // Leitura Giroscópio
    readRegisters(OUTX_L_G, buffer, 6);
    int16_t rawGX = (buffer[1] << 8) | buffer[0];
    int16_t rawGY = (buffer[3] << 8) | buffer[2];
    int16_t rawGZ = (buffer[5] << 8) | buffer[4];

    float sensGiro = 0.00875f; // Fundo de escala +-250 dps (retorna em graus/s)
    gx = rawGX * sensGiro;
    gy = rawGY * sensGiro;
    gz = rawGZ * sensGiro;
}

void IMU::update() {
    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;

    if (dt <= 0.001f || dt > 0.1f) return; // Proteção contra overflow/saltos

    float ax, ay, az, gx, gy, gz;
    readRawData(ax, ay, az, gx, gy, gz);

    // Remove offset do Giroscópio
    gx -= gyroOffset.gx;
    gy -= gyroOffset.gy;
    gz -= gyroOffset.gz;

    // Aplica filtro passa-baixo nas leituras brutas
    ax = lpAccX.apply(ax);
    ay = lpAccY.apply(ay);
    az = lpAccZ.apply(az);
    gx = lpGyrX.apply(gx);
    gy = lpGyrY.apply(gy);
    gz = lpGyrZ.apply(gz);

    // Converte acelerômetro para Gs apenas para a trigonometria do Roll/Pitch
    float ax_g = ax / 9.81f;
    float ay_g = ay / 9.81f;
    float az_g = az / 9.81f;

    // Cálculo estático de Roll e Pitch (em graus) puramente pelo acelerômetro
    float roll_deg = atan2f(ay_g, az_g) * RAD2DEG;
    float pitch_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD2DEG;

    // Integração do Yaw (Giroscópio puro em graus/s)
    float yaw_deg = (yaw * RAD2DEG) + (gz * dt);
    
    // Wrap Yaw para [-180, 180]
    if (yaw_deg > 180.0f) yaw_deg -= 360.0f;
    if (yaw_deg < -180.0f) yaw_deg += 360.0f;

    // Atualiza variáveis da classe em Radianos (padrão esperado pelo EKF)
    roll  = roll_deg * DEG2RAD;
    pitch = pitch_deg * DEG2RAD;
    yaw   = yaw_deg * DEG2RAD;

    // Salva o GyroZ filtrado e sem bias, em rad/s (para o EKF)
    gyroZ_filtered = gz * DEG2RAD;

    // Remoção da gravidade para aceleração longitudinal
    float ax_real, ay_real, az_real;
    removeGravidade(ax, ay, az, roll, pitch, ax_real, ay_real, az_real);
    accelLongitudinal = ax_real;
    accelCentrifuga = ay_real;
}

void IMU::removeGravidade(float ax, float ay, float az, float roll_rad, float pitch_rad, float &ax_real, float &ay_real, float &az_real) {
    // Projeta o vetor gravidade (9.81) no frame do sensor
    float gx_grav = sinf(pitch_rad) * 9.81f;
    float gy_grav = sinf(roll_rad) * cosf(pitch_rad) * 9.81f;
    float gz_grav = cosf(roll_rad) * cosf(pitch_rad) * 9.81f;

    // Remove a componente gravitacional
    ax_real = ax - gx_grav;
    ay_real = ay - gy_grav;
    az_real = az - gz_grav;
}