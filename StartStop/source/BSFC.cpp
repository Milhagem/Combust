#include "BSFC.hpp" 

// =========================================================
// INSTANCIAÇÃO DAS VARIÁVEIS ESTÁTICAS
// =========================================================
volatile unsigned long Bsfc::tempoAnterior = 0;
volatile unsigned long Bsfc::rpm_calculado = 0;
volatile unsigned long Bsfc::hw_deltaTAnterior = 0;
int Bsfc::hw_contadorDentes = 0;
int Bsfc::hw_dentesAcumulados = 0;
volatile unsigned long Bsfc::hw_tempoUltimaVolta = 0;
volatile unsigned long Bsfc::hw_tempoAcumulado = 0;

// =========================================================
// INTERRUPÇÃO POR CONTAGEM DE DENTES (11 DENTES)
// =========================================================
void IRAM_ATTR Bsfc::lerCKP() {
    unsigned long tempoAtualUs = micros();
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

    // Quando bater 11 dentes, considera 1 volta completa
    if (hw_contadorDentes >= 11) {
        unsigned long tempoVolta = tempoAtualUs - hw_tempoUltimaVolta;
        hw_tempoUltimaVolta = tempoAtualUs;
        hw_contadorDentes = 0; // Reseta para a próxima volta

        // Filtro de sanidade física (ignora RPMs absurdos acima de 10.000 ou abaixo de 300)
        if (tempoVolta > 6000 && tempoVolta < 200000) {
            rpm_calculado = 60000000 / tempoVolta;
        }
    }
}

// =========================================================
// INICIALIZAÇÃO E ATUALIZAÇÃO DO FILTRO
// =========================================================
void Bsfc::init() {
    pinMode(SensorCKP, INPUT_PULLUP); 
    attachInterrupt(digitalPinToInterrupt(SensorCKP), lerCKP, FALLING);
    Serial.println("✅ Sensor CKP Iniciado (Interrupcao 11 Dentes)");
}

void Bsfc::atualizaKalman(float mea_e, float est_e, float q) {
    filtroKalmanRPM.setMeasurementError(mea_e);
    filtroKalmanRPM.setEstimateError(est_e);
    filtroKalmanRPM.setProcessNoise(q);
}

// =========================================================
// LEITURA DOS SENSORES (COM FILTRO KALMAN)
// =========================================================
float Bsfc::analisaRPM(){                   
  static unsigned long tempoUltimoFiltro = 0;

  if (micros() - tempoAnterior > 1000000) {
    rpm_calculado = 0;
    // Força a queda suave do filtro a 50Hz se o motor apagar
    if (millis() - tempoUltimoFiltro >= 20) {
        rpm = filtroKalmanRPM.updateEstimate(0); 
        tempoUltimoFiltro = millis();
    }
  } else {
    // A MÁGICA: Só alimenta o filtro a cada 20ms. 
    // Assim o Kalman tem tempo de "ver" a inércia mecânica real do motor.
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
  const float valorVMin = 100.0;  
  const float valorVMax = 4095.0;  
  const float valorLambdaMin = 0.5;  
  const float ValorLambdaMax = 2.8;   
  float valorInicial = analogRead(pin02);
  lambda = valorLambdaMin + ((valorInicial-valorVMin)/(valorVMax-valorVMin))*(ValorLambdaMax-valorLambdaMin);
  if (lambda < valorLambdaMin) lambda = valorLambdaMin;
  if (lambda > ValorLambdaMax) lambda = ValorLambdaMax;
  return lambda;
}
