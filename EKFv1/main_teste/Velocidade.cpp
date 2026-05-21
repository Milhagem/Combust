#include "Velocidade.hpp"

Velocidade::Velocidade(uint8_t pino_sensor, bool habilitarHall)
    : pino(pino_sensor),
      hallHabilitado(habilitarHall),
      periodo(0),
      ultimoTempo(0),
      ultimoDebounce(0),
      rpmAtual(0.0),
      rpmAntigo(0.0)
{
}

void Velocidade::setHallHabilitado(bool habilitado) {
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

bool Velocidade::isHallHabilitado() const {
    return hallHabilitado;
}

void Velocidade::calc() {
    if (!hallHabilitado) return;   // no-op: sensor não conectado

    unsigned long agora = micros();

    // Debounce: ignora pulsos com menos de 2 ms de separação
    if (agora - ultimoDebounce < 2000UL) return;

    rpmAntigo      = rpmAtual;
    periodo        = agora - ultimoTempo;
    ultimoTempo    = agora;
    ultimoDebounce = agora;

    if (periodo > 0) {
        double rpm = (60000000.0 / (double)periodo) / N_IMAS;
        // Rejeita saltos > 750 RPM (ruído de borda)
        if (fabs(rpm - rpmAntigo) > 750.0 && rpmAntigo != 0.0)
            rpmAtual = rpmAntigo;
        else
            rpmAtual = rpm;
    }
}

double Velocidade::getVelocidadeHALL() {
    if (!hallHabilitado) return 0.0;   // sensor ausente → retorna 0 ao EKF

    unsigned long agora = micros();
    unsigned long uTempo;
    double rpm;

    noInterrupts();
    uTempo = ultimoTempo;
    rpm    = rpmAtual;
    interrupts();

    // Timeout 5 s: carro parado
    if ((agora - uTempo) > 5000000UL && rpm < 300.0) {
        noInterrupts();
        rpmAtual = 0.0;
        periodo  = 0;
        interrupts();
        return 0.0;
    }

    return rpm * CIRCUNF_RODA * 0.06;   // RPM → m/s
}

double Velocidade::getVelocidadeHALL_kmh() {
    return getVelocidadeHALL() * MS2_TO_KMHS;
}

float Velocidade::gParaMs2(float accel_g) {
    return accel_g * (float)G_ACEL;
}