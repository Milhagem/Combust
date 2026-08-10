#include "Hall.hpp"

void Hall::Inicializa_Hall(){
    pinMode(PIN_SENSOR_HALL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_HALL), Hall::calc, FALLING);
}

// =================================================================
// GETTERS E SETTERS
// =================================================================
void Hall::setVelocidade(float &vel) { velocidade = vel; }
float Hall::getVelocidade() { return velocidade; }

void Hall::setAceleracao(float &acel) { aceleracao = acel; }
float Hall::getAceleracao() { return aceleracao; }

float Hall::getRPM() { return RPM; }
void Hall::incrementaPulsos() { pulsos++; }

// =================================================================
// FILTRO ANTI-SALTO MECÂNICO
// =================================================================
float Hall::filtroVelocVariacoesGrandes(float velocidadeOld, float velocidadeNew) {
    if (fabs(velocidadeNew - velocidadeOld) > 20.0f) {
        return velocidadeOld;
    }
    return velocidadeNew;
}

// =================================================================
// INTERRUPÇÃO DO SENSOR HALL (ALTA PRIORIDADE NA RAM)
// =================================================================
void IRAM_ATTR Hall::calc() {   
    uint64_t tempoAtual = esp_timer_get_time();
    uint64_t intervaloTemp = tempoAtual - lastPulseInterval;
  
    // Debounce de 10.000 microssegundos (10ms)
    if (intervaloTemp > 10000) { 
        portENTER_CRITICAL_ISR(&mux);
        lastPulseInterval = tempoAtual;

        // Se o intervalo for <= 1.5s, é movimento real.
        // Se for maior, o carro estava parado. Ignoramos esse pulso porque
        // ele mede o "tempo parado" e não a velocidade da roda.
        if (intervaloTemp <= 1500000) {
            pulseInterval = intervaloTemp;
            pulseIntervals[pulseIndex] = pulseInterval;
            pulseIndex = (pulseIndex + 1) % SAMPLE_SIZE; 
        }
        
        portEXIT_CRITICAL_ISR(&mux);
    }
}

// =================================================================
// CÁLCULO DA DINÂMICA DO VEÍCULO (NO LOOP)
// =================================================================
float Hall::update() {
    if(millis() - lastTimerTax >= TAXA_ATUALIZACAO_VEL) {
        lastTimerTax = millis();

        // 1. VERIFICAÇÃO DE PARADA
        if (esp_timer_get_time() - lastPulseInterval > 1500000) { 
            portENTER_CRITICAL(&mux);
            for(int i = 0; i < SAMPLE_SIZE; i++) {
                pulseIntervals[i] = 0; // Limpamos com 0 para não sujar a média
            }
            pulseIndex = 0;
            portEXIT_CRITICAL(&mux);

            RPM = 0.0f;
            velocidade = 0.0f;
            aceleracao = 0.0f;
            return velocidade;
        }

        // 2. CÓPIA SEGURA DA INTERRUPÇÃO
        uint64_t copiaPulseIntervals[SAMPLE_SIZE];
        portENTER_CRITICAL(&mux); 
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            copiaPulseIntervals[i] = pulseIntervals[i];
        }
        portEXIT_CRITICAL(&mux);

        uint64_t averagePulseIntervalUs = 0;
        int amostrasValidas = 0;

        // 3. MÉDIA MÓVEL (Agora ela ignora os zeros da parada)
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            if (copiaPulseIntervals[i] > 0) { // Só faz média do que tem velocidade!
                averagePulseIntervalUs += copiaPulseIntervals[i];
                amostrasValidas++;
            }
        }

        // Se não tem amostra válida, mantém a velocidade atual
        if (amostrasValidas == 0) { return velocidade; } 
        averagePulseIntervalUs /= amostrasValidas;

        // 4. MATEMÁTICA E KALMAN
        velocOld = velocidade;
        
        RPM = 60000000.0f / (PULSOS_POR_VOLTA * averagePulseIntervalUs);
        float velocidadeBruta = RPM * CIRCUNF_RODA * 0.06f; 
        
        velocidade = filtro_Kalman_Vel.aplicar(velocidadeBruta);
        velocidade = filtroVelocVariacoesGrandes(velocOld, velocidade);

        float delta_tempo_segundos = TAXA_ATUALIZACAO_VEL / 1000.0f;
        aceleracao = (((velocidade - velocOld) / MPS_TO_KMPH_FACTOR) / delta_tempo_segundos);
        
        return velocidade;
    } 
    return velocidade; 
}

/*
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
}*/