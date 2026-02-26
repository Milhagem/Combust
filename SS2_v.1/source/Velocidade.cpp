#include "Velocidade.hpp"

#define nIMAS 2.0
#define CircunferenciaRoda 1.81 
#define G_ACEL 9.81  // Aceleração da gravidade em m/s²
#define MS2_TO_KMHS 3.6  // Fator de conversão de m/s para km/h

#include <cmath>

Velocidade::Velocidade(uint8_t pino_sensor)
    : pino(pino_sensor),
      periodo(0),
      ultimoTempo(0),
      ultimoDebounce(0),
      rpmAtual(0),
      rpmAntigo(0),
      ultimaVelocidade(0),
      ultimoTempoAcc(micros()),
      velocidadeIntegrada(0.0),
      kalman(0.0005, 25.0)
{
}

void Velocidade::calc() {
    unsigned long tempoAtual = micros();

    if (tempoAtual - this->ultimoDebounce > 2000) { 
        this->rpmAntigo = this->rpmAtual;
        this->periodo = tempoAtual - this->ultimoTempo;
        this->ultimoTempo = tempoAtual;
        this->ultimoDebounce = tempoAtual;

        if (this->periodo > 0) {
            double rpmCalculado = (60000000.0 / (double)this->periodo) / nIMAS;
            if (fabs(rpmCalculado - this->rpmAntigo) > 750.0 && this->rpmAntigo != 0) {
                this->rpmAtual = this->rpmAntigo;
            } else {
                this->rpmAtual = rpmCalculado;
            }
        }
    }
}

double Velocidade::getVelocidadeHALL() {
    unsigned long agora = micros();
    double rpmRetorno;
    unsigned long uTempo;

    noInterrupts();
    uTempo = ultimoTempo;
    rpmRetorno = rpmAtual;
    interrupts();

    // Timeout de 5 segundos
    if (agora - uTempo > 5000000 && rpmRetorno < 300.0) {
        noInterrupts();
        rpmAtual = 0;
        periodo = 0;
        interrupts();
        rpmRetorno = 0;
    }
    
    return (rpmRetorno * CircunferenciaRoda * 0.06);
}

void Velocidade::atualizaComMultiplasMedidas(double aceleracao_icm, double dt) {
    // Converte aceleração de g para m/s²
    double aceleracao_ms2 = aceleracao_icm * G_ACEL;
    
    // Integra aceleração para obter velocidade
    velocidadeIntegrada += aceleracao_ms2 * dt;
    
    // Obtém velocidade do RPM
    double velocidade_rpm = getVelocidadeHALL();
    
    // Predição: usa aceleração do ICM + aceleração derivada
    kalman.predict(aceleracao_ms2, dt);
    
    // Atualização: funde velocidade RPM + velocidade integrada + aceleração ICM
    kalman.updateComMultiplasMedidas(velocidade_rpm, velocidadeIntegrada, aceleracao_ms2);
}

double Velocidade::getVelocidadeKmh() const {
    // Obtém a velocidade em m/s do Kalman e converte para km/h
    double velo_ms = kalman.getVelocidade();
    return velo_ms * MS2_TO_KMHS;
}

double Velocidade::getAceleracaoMs2() const {
    // Retorna a aceleração estimada em m/s²
    return kalman.getAceleracao();
}
