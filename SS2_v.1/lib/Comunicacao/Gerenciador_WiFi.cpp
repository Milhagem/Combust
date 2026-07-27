#include "Gerenciador_WiFi.hpp"

void Gerenciador_WiFi::conectar_WiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
}

bool Gerenciador_WiFi:: wifi_conectado() {
    return (WiFi.status() == WL_CONNECTED);
}
