#ifndef GPS_HPP
#define GPS_HPP

#include <Arduino.h>
#include <TinyGPS++.h>

class GPS {
private:
    HardwareSerial gpsSerial;
    TinyGPSPlus gps;

    // Configurações de conexão
    int _rxPin;
    int _txPin;
    uint32_t _baudRate;

    // Variáveis de estado
    float currentLat;
    float currentLon;
    float currentAltitude;
    float currentSpeed;

public:
    // Construtor com valores padrão baseados no seu Sensor.hpp original
    // Usa a Serial 2 (padrão do ESP32) por default
    GPS(int rxPin = 17, int txPin = 18, uint32_t baudRate = 9600, uint8_t serialPort = 2);

    // Inicializa a comunicação Serial com o módulo GPS
    void begin();

    // Lê os dados da Serial e atualiza as variáveis internas (chamar no loop)
    void update();

    // Getters públicos (retornam os últimos valores válidos)
    float getLatitude()  const { return currentLat; }
    float getLongitude() const { return currentLon; }
    float getAltitude()  const { return currentAltitude; } // Em metros
    float getSpeed()     const { return currentSpeed; }    // Em m/s
};

#endif // GPS_HPP