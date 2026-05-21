#include "Telemetria.hpp"

Telemetria::Telemetria()
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
      espClient(),
      client(espClient),
      ssid("Diogo's Galaxy M62"),
      password("awur7323"),
      mqtt_server("broker.hivemq.com"),
      lastMsg(0),
      payload{}
{
}

// ================= ICM20948 ===================
void Telemetria::inicializaICM() {
    if (!imu.init()) {
        Serial.println("ICM20948 não encontrado!");
        while (1);
    }
    imu.setAccRange(ICM20948_ACCEL_RANGE_2G);
    imu.setGyrRange(ICM20948_GYRO_RANGE_250);
}

void Telemetria::atualizaSensores() {
    xyzFloat accRaw = imu.getAccRawValues();
    xyzFloat gyrRaw = imu.getGyrRawValues();
    xyzFloat magRaw = imu.getMagRawValues();

    float ax = accRaw.x / 16384.0f * 9.81f;
    float ay = accRaw.y / 16384.0f * 9.81f;
    float az = accRaw.z / 16384.0f * 9.81f;

    float gx = gyrRaw.x / 131.0f * DEG2RAD;
    float gy = gyrRaw.y / 131.0f * DEG2RAD;
    float gz = gyrRaw.z / 131.0f * DEG2RAD;

    float mx = magRaw.x;
    float my = magRaw.y;
    float mz = magRaw.z;

    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1e6f;
    lastMicros = currentMicros;

    if (dt > 0.001f && dt < 0.1f) {
        processaKalman(ax, ay, az, gx, gy, gz, mx, my, mz, dt);
    }
}

void Telemetria::processaKalman(float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float mx, float my, float mz, float dt) {
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

    // Rotacionar aceleração para frame global
    float ax_global = ax * cosf(pitch) * cosf(yaw) - ay * sinf(roll) * sinf(pitch) * cosf(yaw) - az * cosf(roll) * sinf(pitch) * cosf(yaw);
    float ay_global = ax * cosf(pitch) * sinf(yaw) - ay * sinf(roll) * sinf(pitch) * sinf(yaw) - az * cosf(roll) * sinf(pitch) * sinf(yaw);
    float az_global = ax * sinf(pitch) + ay * sinf(roll) * cosf(pitch) + az * cosf(roll) * cosf(pitch);

    accelLongitudinal = ax_global;

    // Atualização Kalman com aceleração
    kalRoll.update(atan2f(ay, az));
    kalPitch.update(atan2f(-ax, sqrtf(ay * ay + az * az)));
    kalYaw.update(atan2f(my, mx));

    roll  = 0.9f * roll  + 0.1f * kalRoll.x;
    pitch = 0.9f * pitch + 0.1f * kalPitch.x;
    yaw   = 0.9f * yaw   + 0.1f * kalYaw.x;
}

void Telemetria::removeGravidade(float ax, float ay, float az,
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
void Telemetria::inicializaGPS() {
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void Telemetria::atualizaGPS() {
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

// ================= CONEXÃO WI-FI ===================
void Telemetria::setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Conectando a ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi conectado!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());
    client.setServer(mqtt_server, 1883);
}

// ======== RECONEXÃO MQTT ========
void Telemetria::reconnect() {
    if (!client.connected()) {
        unsigned long now = millis();
        if (now - lastMsg > 5000) {
            lastMsg = now;
            Serial.print("Tentando conexão MQTT...");
            String clientId = "Milhagem-Device-" + String(random(0xffff), HEX);
            if (client.connect(clientId.c_str())) {
                Serial.println("Conectado ao broker!");
            } else {
                Serial.print("falhou, rc=");
                Serial.print(client.state());
                Serial.println(" - tentando novamente no próximo ciclo");
            }
        }
    }
}

// ======== ENVIO WIFI ========
void Telemetria::EnviodadosWifi() {
    reconnect();
    if (client.connected()) {
        snprintf(payload, sizeof(payload), 
                 "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"accelLong\":%.2f,\"lat\":%.6f,\"lon\":%.6f}",
                 roll, pitch, yaw, accelLongitudinal, currentLat, currentLon);
        
        client.publish("milhagem/sensores", payload);
    }
}