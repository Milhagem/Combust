#include "Sensor.hpp"

constexpr float Sensor::magOffset[3];
constexpr float Sensor::magSoftIron[3][3];

// ================= CONSTRUTOR ===================
Sensor::Sensor(uint8_t pino_sensor, bool habilitarHall)
    : imu(ICM_ADDR),
      kalRoll(0.3f, 0.0f),
      kalPitch(0.3f, 0.1f),
      kalYaw(0.05f, 0.1f),
      lastMicros(0),
      roll(0.0f),
      pitch(0.0f),
      yaw(0.0f),
      accelLongitudinal(0.0f),
      gpsSerial(2),
      pino(pino_sensor),
      hallHabilitado(habilitarHall),
      periodo(0),
      ultimoTempo(0),
      ultimoDebounce(0),
      rpmAtual(0.0),
      rpmAntigo(0.0)
{
}

// ================= ICM20948 ===================
void Sensor::inicializaICM() {
    if (!imu.init()) {
        Serial.println("ICM20948 não encontrado!");
        while (1);
    }
    imu.setAccRange(ICM20948_ACCEL_RANGE_2G);
    imu.setGyrRange(ICM20948_GYRO_RANGE_250);
}

void Sensor::calibrarGiroscopio(uint16_t amostras) {
    Serial.println("Iniciando calibração do giroscópio...");
    Serial.println("MANTENHA O SENSOR COMPLETAMENTE PARADO!");
    
    gyroOffset.gx = 0.0f;
    gyroOffset.gy = 0.0f;
    gyroOffset.gz = 0.0f;
    
    // Coletar múltiplas amostras
    for (uint16_t i = 0; i < amostras; i++) {
        xyzFloat gyrRaw = imu.getGyrRawValues();
        
        // Acumular valores brutos
        gyroOffset.gx += gyrRaw.x;
        gyroOffset.gy += gyrRaw.y;
        gyroOffset.gz += gyrRaw.z;
        
        // Feedback visual/audio
        if (i % 100 == 0) {
            Serial.print(".");
        }
        delay(10);  // ~5 segundos de calibração total
    }
    
    // Média dos valores
    gyroOffset.gx /= amostras;
    gyroOffset.gy /= amostras;
    gyroOffset.gz /= amostras;
    
    Serial.println(" ✓ Calibração concluída!");
    Serial.printf("Offset X: %.2f, Y: %.2f, Z: %.2f\n", 
                  gyroOffset.gx, gyroOffset.gy, gyroOffset.gz);
}

void Sensor::atualizaSensores() {
    xyzFloat accRaw = imu.getAccRawValues();
    xyzFloat gyrRaw = imu.getGyrRawValues();
    xyzFloat magRaw = imu.getMagRawValues();

    // Converter para unidades físicas
    float ax = accRaw.x / 16384.0f * 9.81f;
    float ay = accRaw.y / 16384.0f * 9.81f;
    float az = accRaw.z / 16384.0f * 9.81f;

    float gx = (gyrRaw.x - gyroOffset.gx) / 131.0f * DEG2RAD;
    float gy = (gyrRaw.y - gyroOffset.gy) / 131.0f * DEG2RAD;
    float gz = (gyrRaw.z - gyroOffset.gz) / 131.0f * DEG2RAD;

    float mx = magRaw.x;
    float my = magRaw.y;
    float mz = magRaw.z;

    //FILTRO PASSA-BAIXO (o motor faz o carro vibrar muito, o que interfere no icm)
    ax = lpAccX.apply(ax);
    ay = lpAccY.apply(ay);
    az = lpAccZ.apply(az);

    gx = lpGyrX.apply(gx);
    gy = lpGyrY.apply(gy);
    gz = lpGyrZ.apply(gz);

    mx = lpMagX.apply(mx);
    my = lpMagY.apply(my);
    mz = lpMagZ.apply(mz);

    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1e6f;
    lastMicros = currentMicros;

    if (dt > 0.001f && dt < 0.1f) {
        processaKalman(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
    }
}

void Sensor::processaKalman(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            float mx, float my, float mz, float dt) {
    // Aplicar calibração magnética
    applyMagCalibration(mx, my, mz);

    // Integração giroscópio
    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;

    // Limitação de pitch
    if (pitch > M_PI / 2) pitch = M_PI / 2;
    if (pitch < -M_PI / 2) pitch = -M_PI / 2;

    // Wrap yaw
    if (yaw > M_PI) yaw -= 2 * M_PI;
    if (yaw < -M_PI) yaw += 2 * M_PI;

    // Previsão Kalman
    kalRoll.P  += kalRoll.Q;
    kalPitch.P += kalPitch.Q;
    kalYaw.P   += kalYaw.Q;

    // Atualização Kalman com medições
    kalRoll.update(atan2f(ay, az));
    kalPitch.update(atan2f(-ax, sqrtf(ay * ay + az * az)));
    kalYaw.update(atan2f(my, mx));

    roll  = 0.9f * roll  + 0.1f * kalRoll.x;
    pitch = 0.9f * pitch + 0.1f * kalPitch.x;
    yaw   = 0.9f * yaw   + 0.1f * kalYaw.x;

    // Calcular aceleração longitudinal (rotacionada para frame global)
    float ax_global = ax * cosf(pitch) * cosf(yaw) - ay * sinf(roll) * sinf(pitch) * cosf(yaw) - az * cosf(roll) * sinf(pitch) * cosf(yaw);
    accelLongitudinal = ax_global;

    // ── Valores para o AckermanEKF ────────────────────────────
    // gz já tem bias removido (gyroOffset.gz aplicado em atualizaSensores)
    // e passou pelo filtro passa-baixo. Armazena direto.
    gyroZ_filtered = gz;

    // Bearing magnético calibrado: atan2 após calibração hard+soft iron.
    // Aplica declinação magnética local para obter heading verdadeiro (North).
    // mx e my já passaram por applyMagCalibration() nesta chamada.
    float bearing_raw = atan2f(my, mx);
    magBearing = bearing_raw + MAG_DECLINATION;
    // Normaliza para [-pi, pi]
    if (magBearing >  (float)M_PI) magBearing -= 2.0f * (float)M_PI;
    if (magBearing < -(float)M_PI) magBearing += 2.0f * (float)M_PI;
}

void Sensor::applyMagCalibration(float &mx, float &my, float &mz) {
    // Compensação de offset
    mx -= magOffset[0];
    my -= magOffset[1];
    mz -= magOffset[2];

    // Aplicar soft-iron calibration (matriz 3x3)
    float mx_cal = magSoftIron[0][0] * mx + magSoftIron[0][1] * my + magSoftIron[0][2] * mz;
    float my_cal = magSoftIron[1][0] * mx + magSoftIron[1][1] * my + magSoftIron[1][2] * mz;
    float mz_cal = magSoftIron[2][0] * mx + magSoftIron[2][1] * my + magSoftIron[2][2] * mz;

    mx = mx_cal;
    my = my_cal;
    mz = mz_cal;
}

void Sensor::removeGravidade(float ax, float ay, float az,
                             float roll, float pitch,
                             float &ax_real, float &ay_real, float &az_real) {
    float gx = sinf(pitch);
    float gy = sinf(roll) * cosf(pitch);
    float gz = cosf(roll) * cosf(pitch);

    ax_real = ax - gx * 9.81f;
    ay_real = ay - gy * 9.81f;
    az_real = az - gz * 9.81f;
}

// ================= GPS ===================
void Sensor::inicializaGPS() {
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void Sensor::atualizaGPS() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
        currentLat  = gps.location.lat();
        currentLon  = gps.location.lng();
    }

    if (gps.altitude.isUpdated()) {
        currentAltitude = gps.altitude.meters();
    }

    if (gps.speed.isUpdated()) {
        currentSpeed = gps.speed.mps();
    }
}

// ================= HALL ===================
void Sensor::setHallHabilitado(bool habilitado) {
    hallHabilitado = habilitado;
    if (!habilitado) {
        noInterrupts();
        rpmAtual       = 0.0;
        rpmAntigo      = 0.0;
        periodo        = 0;
        ultimoTempo    = 0;
        ultimoDebounce = 0;
        interrupts();
    }
}

bool Sensor::isHallHabilitado() const {
    return hallHabilitado;
}

void Sensor::calc() {
    if (!hallHabilitado) return;

    unsigned long agora = micros();

    if (agora - ultimoDebounce < 2000UL) return;

    rpmAntigo      = rpmAtual;
    periodo        = agora - ultimoTempo;
    ultimoTempo    = agora;
    ultimoDebounce = agora;

    if (periodo > 0) {
        double rpm = (60000000.0 / (double)periodo) / N_IMAS;
        if (fabs(rpm - rpmAntigo) > 750.0 && rpmAntigo != 0.0)
            rpmAtual = rpmAntigo;
        else
            rpmAtual = rpm;
    }
}

double Sensor::getVelocidadeHALL() {
    if (!hallHabilitado) return 0.0;

    unsigned long agora = micros();
    unsigned long uTempo;
    double rpm;

    noInterrupts();
    uTempo = ultimoTempo;
    rpm    = rpmAtual;
    interrupts();

    if ((agora - uTempo) > 5000000UL && rpm < 300.0) {
        noInterrupts();
        rpmAtual = 0.0;
        periodo  = 0;
        interrupts();
        return 0.0;
    }

    return rpm * CIRCUNF_RODA * 0.06;
}

double Sensor::getVelocidadeHALL_kmh() {
    return getVelocidadeHALL() * MS2_TO_KMHS;
}

float Sensor::gParaMs2(float accel_g) {
    return accel_g * (float)G_ACEL;
}