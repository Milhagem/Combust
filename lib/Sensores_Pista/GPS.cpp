#include "GPS.hpp"

// =================================================================
// SETTERS (Configuração)
// =================================================================
void GPS::setConfig(int rxPin, int txPin, uint32_t baudRate) {
    _rxPin = rxPin;
    _txPin = txPin;
    _baudRate = baudRate;
}

// =================================================================
// GETTERS
// =================================================================
float GPS::getLatitude()  { return currentLat; }
float GPS::getLongitude() { return currentLon; }
float GPS::getAltitude()  { return currentAltitude; }
float GPS::getSpeed()     { return currentSpeed; }

// =================================================================
// INICIALIZAÇÃO
// =================================================================
void GPS::begin() {
    // Inicializa a HardwareSerial com as configurações definidas
    gpsSerial.begin(_baudRate, SERIAL_8N1, _rxPin, _txPin);
}

// =================================================================
// ATUALIZAÇÃO (PROCESSAMENTO NMEA)
// =================================================================
void GPS::update() {
    // Lê todos os bytes disponíveis no buffer da Serial e repassa para o TinyGPS++
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // Só atualiza as variáveis internas se o TinyGPS++ confirmar que recebeu um dado novo e válido
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
