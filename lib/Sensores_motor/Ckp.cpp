#include "Ckp.hpp" 

// =========================================================
// INICIALIZAÇÃO DO SENSOR
// =========================================================
void Ckp::Inicializar_setup_sensores_motor() {
    pinMode(SensorCKP, INPUT_PULLUP); 
    
    attachInterrupt(digitalPinToInterrupt(SensorCKP), lerCKP, FALLING);
    
    Serial.println("✅ Sensor CKP Iniciado (Hardware HR-Timer 64-bit - 11 Dentes)");
}

// =========================================================
// INTERRUPÇÃO DE HARDWARE (HIGH-RESOLUTION TIMER 64-BIT)
// =========================================================
void IRAM_ATTR Ckp::lerCKP() {
    uint64_t tempoAtualUs = esp_timer_get_time();
    uint64_t deltaT = tempoAtualUs - tempoAnterior;

    // Timeout: se passar de 1 segundo sem sinal, motor está parado
    if (deltaT > 1000000) { 
        status_motor = false;
        rpm_calculado = 0;
        hw_contadorDentes = 0;
        tempoAnterior = tempoAtualUs;
        hw_tempoUltimaVolta = tempoAtualUs;
        return;
    }

    // Filtro de hardware (Bounce): ignora ruídos menores que 200us
    if (deltaT < 200) return; 

    tempoAnterior = tempoAtualUs;
    hw_contadorDentes++;

    // Lógica mantida: 11 dentes = 1 volta completa
    if (hw_contadorDentes >= 11) {
        unsigned long tempoVoltaUs = tempoAtualUs - hw_tempoUltimaVolta;
        hw_tempoUltimaVolta = tempoAtualUs;
        hw_contadorDentes = 0; // Reseta para a próxima volta

        // Filtro de sanidade física (ignora RPMs absurdos)
        if (tempoVoltaUs > 6000 && tempoVoltaUs < 200000) {
            rpm_calculado = 60000000 / tempoVoltaUs;
        }
    }
}


// =========================================================
// LEITURA DOS SENSORES (COM FILTRO KALMAN A 50HZ)
// =========================================================
float Ckp::analisaRPM(){                   
    static unsigned long tempoUltimoFiltro = 0;

    // essas 4 linhas são para evitar que a leitura do rpm seja feita enquanto a interrupção está atualizando o tempoAnterior
    uint64_t copiaTempoAnterior;
    portENTER_CRITICAL(&mux);
    copiaTempoAnterior = tempoAnterior;
    portEXIT_CRITICAL(&mux);

    // Usa o relógio absoluto também para o timeout suave de parada
   if (esp_timer_get_time() - copiaTempoAnterior > 1000000) {
        rpm_calculado = 0;
        if (millis() - tempoUltimoFiltro >= 20) {
            // Alterado de updateEstimate para aplicar
            rpm = filtroKalmanRPM.aplicar(0); 
            tempoUltimoFiltro = millis();
        }
    } else {
        // Alimenta o filtro a 50Hz (20ms) para dar tempo de inércia
        if (millis() - tempoUltimoFiltro >= 20) {
            // Alterado de updateEstimate para aplicar
            rpm = filtroKalmanRPM.aplicar((float)rpm_calculado); 
            tempoUltimoFiltro = millis();
        }
    }
    return rpm;
}