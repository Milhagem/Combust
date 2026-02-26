#include "Telemetria.hpp"

constexpr float Telemetria::magOffset[3];
constexpr float Telemetria::magSoftIron[3][3];

// ================= CONSTRUTOR ===================
Telemetria::Telemetria()
    : imu(ICM_ADDR),
      kalRoll(0.001f, 0.03f),
      kalPitch(0.001f, 0.03f), //(q, r) quanto maior o q, maior a confiança no giroscópio, quanto maior o r menor a confiança no acelerômetro/magnetômetro
      kalYaw(0.01f, 1.0f),
      lastMicros(0),
      roll(0.0f),
      pitch(0.0f),
      yaw(0.0f),
      accelLongitudinal(0.0f),
      ssid("Diogo's Galaxy M62"),
      password("awur7323"),      
      mqtt_server("broker.hivemq.com"),
      espClient(),  
      client(espClient),     
      lastMsg(0)
{
}

// ================= CALIBRAÇÃO MAGNETÔMETRO ===================
void Telemetria::applyMagCalibration(float &mx, float &my, float &mz) {
    float x = mx - magOffset[0];
    float y = my - magOffset[1];
    float z = mz - magOffset[2];

    mx = magSoftIron[0][0]*x + magSoftIron[0][1]*y + magSoftIron[0][2]*z;
    my = magSoftIron[1][0]*x + magSoftIron[1][1]*y + magSoftIron[1][2]*z;
    mz = magSoftIron[2][0]*x + magSoftIron[2][1]*y + magSoftIron[2][2]*z;
}

// ================= INICIALIZAÇÃO DO ICM ===================
void Telemetria::inicializaICM() {
    if (!imu.init()) {
        Serial.println("ICM20948 nao encontrado!");
        while (1);
    }

    imu.enableAcc(true);
    imu.enableGyr(true);
    imu.initMagnetometer();

    imu.setAccRange(ICM20948_ACC_RANGE_4G);
    imu.setGyrRange(ICM20948_GYRO_RANGE_500);

    imu.setAccDLPF(ICM20948_DLPF_6);
    imu.setGyrDLPF(ICM20948_DLPF_6);

    imu.autoOffsets();

    lastMicros = micros();
}

// ================= LEITURA DOS SENSORES ===================
void Telemetria::atualizaSensores() {
    // Lê os sensores
    xyzFloat acc, gyr, mag;
    
    imu.readSensor();
    acc = imu.getAccRawValues();
    gyr = imu.getGyrValues();
    mag = imu.getMagValues();

    // Converte aceleração de LSB para g
    constexpr float ACC_LSB_PER_G = 8192.0f;
    float ax = acc.x / ACC_LSB_PER_G;
    float ay = acc.y / ACC_LSB_PER_G;
    float az = acc.z / ACC_LSB_PER_G;

    // Aplica calibração do magnetômetro
    applyMagCalibration(mag.x, mag.y, mag.z);

    // Calcula dt
    float dt = (micros() - lastMicros) * 1e-6f;
    lastMicros = micros();

    if (dt <= 0.0f) dt = 0.001f;  // Proteção contra dt inválido

    // Processa Kalman
    processaKalman(ax, ay, az, gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, dt);

    // Remove gravidade e calcula aceleração longitudinal
    float ax_real, ay_real, az_real;
    removeGravidade(ax, ay, az, roll, pitch, ax_real, ay_real, az_real);
    accelLongitudinal = ax_real;
}

// ================= PROCESSAMENTO KALMAN ===================
void Telemetria::processaKalman(float ax, float ay, float az, 
                                 float gx, float gy, float gz, 
                                 float mx, float my, float mz, 
                                 float dt) {
    
    // ===== PREVISÃO (GIROSCÓPIO) =====
    kalRoll.predict(gx * DEG2RAD, dt);
    kalPitch.predict(gy * DEG2RAD, dt);
    kalYaw.predict(gz * DEG2RAD, dt);

    // ===== MEDIÇÕES (ACELERÔMETRO + MAGNETÔMETRO) =====
    float roll_m  = atan2(ay, az);
    float pitch_m = atan2(-ax, sqrt(ay*ay + az*az));
    
    // Cálculo do yaw a partir do magnetômetro
    // Rotaciona a medição de magnetômetro para frame do corpo
    float cr = cos(roll);
    float sr = sin(roll);
    float cp = cos(pitch);
    float sp = sin(pitch);
    
    float mx_body = mx * cp + my * sr * sp + mz * cr * sp;
    float my_body = my * cr - mz * sr;
    
    float yaw_m = atan2(-my_body, mx_body);

    // ===== CORREÇÃO (KALMAN UPDATE) =====
    roll  = kalRoll.update(roll_m);
    pitch = kalPitch.update(pitch_m);
    yaw   = kalYaw.update(yaw_m);
}

// ================= REMOÇÃO DA GRAVIDADE ===================
void Telemetria::removeGravidade(float ax, float ay, float az, 
                                  float roll, float pitch, 
                                  float &ax_real, float &ay_real, float &az_real) {
    
    float cr = cos(roll);
    float sr = sin(roll);
    float cp = cos(pitch);
    float sp = sin(pitch);

    // Componentes de gravidade no frame do acelerômetro
    float gx = -sp;
    float gy = sr * cp;
    float gz = cr * cp;

    // Remove gravidade
    ax_real = ax - gx;
    ay_real = ay - gy;
    az_real = az - gz;
}

// ================= CONEXÂO WI-FI ===================
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

  Serial.println("");
  Serial.println("WiFi conectado!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, 1883);
}

// ======== FUNÇÃO DE RECONEXÃO MQTT ========
void Telemetria::reconnect() {
    if (!client.connected()) {
        unsigned long now = millis();

        // Tenta reconectar apenas a cada 5 segundos para não poluir o log
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

// ======== FAZ O ENVIO DOS DADOS POR MEIO DO WIFI ========
void Telemetria::EnviodadosWifi() {
    reconnect();
    if (client.connected()) {
        client.loop();
        
        snprintf(payload, sizeof(payload),
            "{\"Acelera\":%.2f,\"Inclinacao\":%.2f,\"Rotacao\":%.2f,\"Lat\":%.6f,\"Long\":%.6f}",
            accelLongitudinal, pitch, yaw, 0.0, 0.0);

        Serial.print("Publicando: ");
        Serial.println(payload);
        client.publish("sensor/esp32S3", payload);
    }
}
