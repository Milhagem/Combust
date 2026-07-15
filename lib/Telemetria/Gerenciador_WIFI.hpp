#pragma once
#include <WiFi.h>

class Gerencia_wifi{
    // Uma classe que somente e chamada no setup pois, caso o wifi caia, ele tenta reconectar automaticamente.
    private:
        const char* ssid = "Urban";
        const char* password = "pwtc2026";

    public:
        void conectar_WiFi();
        bool wifi_conectado();

};