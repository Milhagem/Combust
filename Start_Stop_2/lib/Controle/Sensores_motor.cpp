#include "Sensores_motor.hpp" 


float Sensores_motor::analisaLambda(){
  const float valorVMin = 140.0f;
  const float valorVMax = 4000.0f;
  const float valorLambdaMin = 0.5f;
  const float ValorLambdaMax = 2.8f;
  float valorInicial = analogRead(pin02);
  float Lambda_bruto = 0.0f;

  
  Lambda_bruto = valorLambdaMin + ((valorInicial-valorVMin)/(valorVMax-valorVMin))*(ValorLambdaMax-valorLambdaMin);
  
  if (Lambda_bruto < valorLambdaMin) Lambda_bruto = valorLambdaMin;
  if (Lambda_bruto > ValorLambdaMax) Lambda_bruto = ValorLambdaMax;
  
  lambda = filtroExponencialLambda.aplicar(Lambda_bruto);
  return lambda;
}

float Sensores_motor::analisaMap(){
  const float valorMin = 100.0f;
  const float valorMax = 4095.0f;
  float valorInicial = analogRead(pinmap);
  
  if (valorInicial < valorMin){
    valorInicial = valorMin;
    status_map = false;
  }else{
    status_map = true;
  }
  if (valorInicial > valorMax) valorInicial = valorMax;
  
  float porcentagem = ((valorInicial-valorMin)/(valorMax-valorMin)) * 100.0f;
  map = porcentagem;
  return porcentagem;
}


float Sensores_motor::analisaPosBorbo(){   
  const float valorMin = 450.0f;
  const float valorMax = 3500.0f;
  float leituraAtual = analogRead(Pintp);

  
  if (leituraAtual < valorMin){
     leituraAtual = valorMin;
     status_tps = false;
  }else{
    status_tps = true;
  }
  if (leituraAtual > valorMax) leituraAtual = valorMax;

  float porcentagemAtual = ((leituraAtual - valorMin) / (valorMax - valorMin)) * 100.0f;

posborbo = filtroExponencialPosBorbo.aplicar(porcentagemAtual);
  
  return posborbo;
}

void Sensores_motor::analisa_status_central() {
    if (status_map && status_tps) {
        status_central = true;
        tempo_central_desligada = 0; 
    } else {
        if(tempo_central_desligada == 0) {
            tempo_central_desligada = millis();
        } else if (millis() - tempo_central_desligada > 2000) {
            status_central = false; 
        }
    }
}


 Sensores_motor::statesEngine Sensores_motor::analisa_status_motor() {
    if (rpm==0 || tensao < tensaoMotorON) {
       return engineOFF;
    } else {
        return engineON;
    }
}
       

float Sensores_motor::analisaTensao(){
  const float RESOLUCAO_ESP = 4095.0f;
  const float TENSAO_ESP = 3.3f;

  float valorInicial = analogRead(LM2907);
  float tensao_bruta = (valorInicial/RESOLUCAO_ESP)*TENSAO_ESP; 
  tensao = filtroExponencialTensao.aplicar(tensao_bruta);
  return tensao;
}


// =========================================================
// INICIALIZAÇÃO DO SENSOR
// =========================================================
void Sensores_motor::Inicializar_setup_sensores_motor() {
    pinMode(SensorCKP, INPUT_PULLUP); 
    pinMode(LM2907, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(SensorCKP), lerCKP, FALLING);
    
    Serial.println("✅ Sensor CKP Iniciado (Hardware HR-Timer 64-bit - 11 Dentes)");
}

// =========================================================
// INTERRUPÇÃO DE HARDWARE (HIGH-RESOLUTION TIMER 64-BIT)
// =========================================================
void IRAM_ATTR Sensores_motor::lerCKP() {
    // Puxa o relógio absoluto do silício (imune a atrasos de software)
    unsigned long tempoAtualUs = esp_timer_get_time();
    unsigned long deltaT = tempoAtualUs - tempoAnterior;

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
float Sensores_motor::analisaRPM(){                   
    static unsigned long tempoUltimoFiltro = 0;

    // Usa o relógio absoluto também para o timeout suave de parada
    if (esp_timer_get_time() - tempoAnterior > 1000000) {
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