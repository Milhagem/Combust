#ifndef HALL_HPP
#define HALL_HPP

#include <Arduino.h>

class Hall {
private:
    // Configurações do hardware e veículo
    uint8_t _pin;
    float _circunfRoda;
    int _pulsosPorVolta;
    unsigned long _taxaAtualizacao; // ms

    // ── Truque para usar Interrupção (ISR) dentro de uma Classe ──
    static Hall* instancia;
    static void IRAM_ATTR isrStatic();
    void IRAM_ATTR handleInterrupt();

    // ── Variáveis de Estado (Saídas) ──
    float velocidade; // [km/h]
    float aceleracao; // [m/s²]
    float RPM;
    float velocOld;
    unsigned long lastTimerTax;

    // ── Variáveis Voláteis (Compartilhadas com a Interrupção) ──
    static const int SAMPLE_SIZE = 4;
    volatile unsigned long pulseIntervals[SAMPLE_SIZE];
    volatile int pulseIndex;
    volatile unsigned long lastPulseTime;

public:
    // Construtor com os mesmos valores padrão do seu código original
    Hall(uint8_t pin = 1, float circunfRoda = 1.81f, int imas = 5, unsigned long taxaAtualizacao = 100);

    // Inicializa o pino e anexa a interrupção (Chamar no setup)
    void begin();

    // Calcula as médias e atualiza as variáveis (Chamar no loop)
    void update();

    // Getters
    float getVelocidade() const { return velocidade; } // km/h
    float getAceleracao() const { return aceleracao; } // m/s²
    float getRPM()        const { return RPM; }
};

#endif // HALL_HPP