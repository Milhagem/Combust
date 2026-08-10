#pragma once

#include <Arduino.h>
#include <TinyGPS++.h>

class GPS {
private:
    // Inicializa a HardwareSerial na porta 2 (Padrão ESP32)
    inline static HardwareSerial gpsSerial{2}; 
    inline static TinyGPSPlus gps;

    // Configurações de conexão (Valores padrão)
    inline static int _rxPin = 17;
    inline static int _txPin = 18;
    inline static uint32_t _baudRate = 9600;

    // Variáveis de estado
    inline static float currentLat = 0.0f;
    inline static float currentLon = 0.0f;
    inline static float currentAltitude = 0.0f;
    inline static float currentSpeed = 0.0f;

public:
    // Configuração (Substitui o Construtor)
    static void setConfig(int rxPin, int txPin, uint32_t baudRate);

    // Inicializa a comunicação Serial com o módulo GPS
    static void begin();

    // Lê os dados da Serial e atualiza as variáveis internas (chamar no loop)
    static void update();

    // Getters públicos (As implementações foram movidas para o .cpp)
    static float getLatitude();
    static float getLongitude();
    static float getAltitude(); // Em metros
    static float getSpeed();    // Em m/s
};
