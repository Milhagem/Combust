#include "Velocidade.hpp"




void Velocidade:: Inicializar_setup_sensores_velocidade(){
pinMode(PIN_SENSOR_HALL, INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(PIN_SENSOR_HALL), Velocidade::calc, FALLING);
}

// =================================================================
// GETTERS E SETTERS
// =================================================================
void Velocidade::setVelocidade(float &vel) { velocidade = vel; }
float Velocidade::getVelocidade() { return velocidade; }
void Velocidade::setAceleracao(float &aceleracao) { acelera = aceleracao; }
float Velocidade::getAcelera() { return acelera; }
float Velocidade::getRPM() { return RPM; }
void Velocidade::incrementaPulsos() { pulsos++; }

// =================================================================
// FILTRO ANTI-SALTO MECÂNICO
// =================================================================
float Velocidade::filtroVelocVariacoesGrandes(float velocidadeOld, float velocidadeNew) {
    if (fabs(velocidadeNew - velocidadeOld) > 20.0f) {
        return velocidadeOld;
    }
    return velocidadeNew;
}

// =================================================================
// INTERRUPÇÃO DO SENSOR HALL (ALTA PRIORIDADE NA RAM)
// =================================================================
void IRAM_ATTR Velocidade::calc() {   
    uint64_t tempoAtual = esp_timer_get_time();
    uint64_t intervaloTemp = tempoAtual - lastPulseInterval;
  
    // Debounce de 10.000 microssegundos (10ms)
    if (intervaloTemp > 10000) { 
        portENTER_CRITICAL_ISR(&mux);
        pulseInterval = intervaloTemp;
        lastPulseInterval = tempoAtual;
        pulseIntervals[pulseIndex] = pulseInterval;
        pulseIndex = (pulseIndex + 1) % SAMPLE_SIZE; 
        portEXIT_CRITICAL_ISR(&mux);
    }
}

// =================================================================
// CÁLCULO DA DINÂMICA DO VEÍCULO (NO LOOP)
// =================================================================

float Velocidade::calculaVelocidade() {
    if(millis() - lastTimerTax >= TAXA_ATUALIZACAO_VEL) {
        lastTimerTax = millis();

        if (esp_timer_get_time() - lastPulseInterval > 1500000) { 
            portENTER_CRITICAL(&mux);
            for(int i = 0; i < SAMPLE_SIZE; i++) {
                pulseIntervals[i] = 1500000; 
            }
            portEXIT_CRITICAL(&mux);

            RPM = 0.0f;
            velocidade = 0.0f;
            acelera = 0.0f;
            return velocidade;
        }

       uint64_t copiaPulseIntervals[SAMPLE_SIZE];
        portENTER_CRITICAL(&mux); 
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            copiaPulseIntervals[i] = pulseIntervals[i];
        }
        portEXIT_CRITICAL(&mux);

        // 3. MÉDIA MÓVEL
        uint64_t averagePulseIntervalUs = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            averagePulseIntervalUs += copiaPulseIntervals[i];
        }
        averagePulseIntervalUs /= SAMPLE_SIZE;

        if (averagePulseIntervalUs == 0) { return velocidade; } 

        // 4. MATEMÁTICA E KALMAN
        velocOld = velocidade;
        
        RPM = 60000000.0f / (PULSOS_POR_VOLTA * averagePulseIntervalUs);
        float velocidadeBruta = RPM * CIRCUNF_RODA * 0.06f; 
        
        velocidade = filtro_Kalman_Vel.aplicar(velocidadeBruta);
        velocidade = filtroVelocVariacoesGrandes(velocOld, velocidade);

        float delta_tempo_segundos = TAXA_ATUALIZACAO_VEL / 1000.0f;
        acelera = (((velocidade - velocOld) / MPS_TO_KMPH_FACTOR) / delta_tempo_segundos);
        
        return velocidade;
    } 
    return velocidade; 
}