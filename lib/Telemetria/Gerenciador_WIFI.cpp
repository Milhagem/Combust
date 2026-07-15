#include "Gerenciador_wifi.hpp"




void Gerencia_wifi::conectar_WiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
}

bool Gerencia_wifi:: wifi_conectado() {
    return (WiFi.status() == WL_CONNECTED);
}


    
