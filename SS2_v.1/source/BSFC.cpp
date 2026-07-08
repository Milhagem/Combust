#include "BSFC.hpp" 

// =========================================================
// INSTANCIAÇÃO DAS VARIÁVEIS ESTÁTICAS DA CLASSE
// =========================================================
volatile unsigned long Bsfc::tempoAnterior = 0;
volatile unsigned long Bsfc::rpm_calculado = 0;
volatile unsigned long Bsfc::hw_deltaTAnterior = 0;
int Bsfc::hw_contadorDentes = 0;
int Bsfc::hw_dentesAcumulados = 0;
volatile unsigned long Bsfc::hw_tempoUltimaVolta = 0;
volatile unsigned long Bsfc::hw_tempoAcumulado = 0;

// =========================================================
// INTERRUPÇÃO DE HARDWARE (HIGH-RESOLUTION TIMER 64-BIT)
// =========================================================
void IRAM_ATTR Bsfc::lerCKP() {
    // Puxa o relógio absoluto do silício (imune a atrasos de software)
    unsigned long tempoAtualUs = esp_timer_get_time();
    unsigned long deltaT = tempoAtualUs - tempoAnterior;

    // Timeout: se passar de 1 segundo sem sinal, motor está parado
    if (deltaT > 1000000) { 
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
// INICIALIZAÇÃO DO SENSOR
// =========================================================
void Bsfc::init() {
    pinMode(SensorCKP, INPUT_PULLUP); 
    
    // Anexa a interrupção priorizando a RAM rápida (IRAM) para disparo instantâneo
    attachInterrupt(digitalPinToInterrupt(SensorCKP), lerCKP, FALLING);
    
    Serial.println("✅ Sensor CKP Iniciado (Hardware HR-Timer 64-bit - 11 Dentes)");
}

void Bsfc::atualizaKalman(float mea_e, float est_e, float q) {
    filtroKalmanRPM.setMeasurementError(mea_e);
    filtroKalmanRPM.setEstimateError(est_e);
    filtroKalmanRPM.setProcessNoise(q);
}

// =========================================================
// LEITURA DOS SENSORES (COM FILTRO KALMAN A 50HZ)
// =========================================================
float Bsfc::analisaRPM(){                   
  static unsigned long tempoUltimoFiltro = 0;

  // Usa o relógio absoluto também para o timeout suave de parada
  if (esp_timer_get_time() - tempoAnterior > 1000000) {
    rpm_calculado = 0;
    if (millis() - tempoUltimoFiltro >= 20) {
        rpm = filtroKalmanRPM.updateEstimate(0); 
        tempoUltimoFiltro = millis();
    }
  } else {
    // Alimenta o filtro a 50Hz (20ms) para dar tempo de inércia
    if (millis() - tempoUltimoFiltro >= 20) {
        rpm = filtroKalmanRPM.updateEstimate((float)rpm_calculado); 
        tempoUltimoFiltro = millis();
    }
  }
  return rpm;
}

float Bsfc::analisaPosBorbo(){   
  const float valorMin = 450;
  const float valorMax = 3500.0;
  const float ALFA = 0.05; 
  float leituraAtual = analogRead(Pintp);
  
  if (leituraAtual < valorMin) leituraAtual = valorMin;
  if (leituraAtual > valorMax) leituraAtual = valorMax;
  
  float porcentagemAtual = ((leituraAtual - valorMin) / (valorMax - valorMin)) * 100.0;
  if (primeiraLeituraTp) { posborbo = porcentagemAtual; primeiraLeituraTp = false; } 
  else { posborbo = (ALFA * porcentagemAtual) + ((1.0 - ALFA) * posborbo); }
  
  return posborbo;
}

float Bsfc::analisaMap(){
  const float valorMin = 100.0;
  const float valorMax = 4095.0;
  float valorInicial = analogRead(pinmap);
  
  if (valorInicial < valorMin) valorInicial = valorMin;
  if (valorInicial > valorMax) valorInicial = valorMax;
  
  float porcentagem = ((valorInicial-valorMin)/(valorMax-valorMin)) * 100;
  map = porcentagem;
  return porcentagem; 
}

float Bsfc::analisaLambda(){ 
  const float valorVMin = 140.0;  
  const float valorVMax = 4000.0;  
  const float valorLambdaMin = 0.5;  
  const float ValorLambdaMax = 2.8;   
  float valorInicial = analogRead(pin02);
  
  lambda = valorLambdaMin + ((valorInicial-valorVMin)/(valorVMax-valorVMin))*(ValorLambdaMax-valorLambdaMin);
  
  if (lambda < valorLambdaMin) lambda = valorLambdaMin;
  if (lambda > ValorLambdaMax) lambda = ValorLambdaMax;
  
  return lambda;
}