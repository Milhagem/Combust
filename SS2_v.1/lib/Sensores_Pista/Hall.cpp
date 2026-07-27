#include "Hall.hpp"

// Inicializa o ponteiro estático fora da classe
Hall* Hall::instancia = nullptr;

// ==========================================
// Construtor
// ==========================================
Hall::Hall(uint8_t pin, float circunfRoda, int imas, unsigned long taxaAtualizacao)
    : _pin(pin), _circunfRoda(circunfRoda), _pulsosPorVolta(imas), _taxaAtualizacao(taxaAtualizacao),
      velocidade(0.0f), aceleracao(0.0f), RPM(0.0f), velocOld(0.0f), lastTimerTax(0),
      pulseIndex(0), lastPulseTime(0)
{
    // Grava o endereço deste objeto no ponteiro estático para a ISR conseguir enxergá-lo
    instancia = this; 
    
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        pulseIntervals[i] = 0;
    }
}

// ==========================================
// Inicialização
// ==========================================
void Hall::begin() {
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), isrStatic, FALLING);
}

// ==========================================
// Interrupção (ISR)
// ==========================================
void IRAM_ATTR Hall::isrStatic() {
    if (instancia != nullptr) {
        instancia->handleInterrupt();
    }
}

void IRAM_ATTR Hall::handleInterrupt() {
    unsigned long tempoAtual = millis();
    unsigned long intervaloTemp = tempoAtual - lastPulseTime;
    
    // Debounce: só aceita o pulso se passou mais de 10ms desde o último
    if (intervaloTemp > 10) { 
        pulseIntervals[pulseIndex] = intervaloTemp;
        pulseIndex = (pulseIndex + 1) % SAMPLE_SIZE; 
        lastPulseTime = tempoAtual;
    }
}

// ==========================================
// Cálculo Matemático (Chamar no Loop)
// ==========================================
void Hall::update() {
    unsigned long tempoAtual = millis();

    // Só calcula se passou o tempo da taxa de atualização (100ms por padrão)
    if (tempoAtual - lastTimerTax >= _taxaAtualizacao) {
        lastTimerTax = tempoAtual;

        // 1. TIMEOUT DE PARADA: Se passou muito tempo (1.5s) sem pulso, força tudo para zero
        if (tempoAtual - lastPulseTime > 1500) { 
            RPM = 0.0f;
            velocidade = 0.0f;
            aceleracao = 0.0f;
            
            for(int i = 0; i < SAMPLE_SIZE; i++) {
                pulseIntervals[i] = 0;
            }
            return; // Sai da função, tudo já está zerado
        }

        // 2. CÓPIA SEGURA: Pausa as interrupções apenas pelo tempo de copiar o array
        unsigned long copiaPulseIntervals[SAMPLE_SIZE];
        
        noInterrupts(); 
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            copiaPulseIntervals[i] = pulseIntervals[i];
        }
        interrupts();

        // 3. CÁLCULO DA MÉDIA MÓVEL (usando a cópia segura)
        unsigned long averagePulseInterval = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            averagePulseInterval += copiaPulseIntervals[i];
        }
        averagePulseInterval /= SAMPLE_SIZE;

        if (averagePulseInterval == 0) return;

        // 4. CONVERSÕES FINAIS
        float delta_tempo_segundos = _taxaAtualizacao / 1000.0f;
        velocOld = velocidade;
        
        RPM = 60000.0f / (_pulsosPorVolta * averagePulseInterval);
        velocidade = RPM * _circunfRoda * 0.06f; // Em km/h
        aceleracao = (((velocidade - velocOld) / 3.6f) / delta_tempo_segundos); // Em m/s²
    }
}
