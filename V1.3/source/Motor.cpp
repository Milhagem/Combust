#include "Motor.h"


float Motor::analisaTensao(){
  const float RESOLUCAO_ARDUINO = 1024.0;
  const float TENSAO_ARDUINO = 5.0;

  float valorInicial = analogRead(LM2907); 
  float tensao = (valorInicial/RESOLUCAO_ARDUINO)*TENSAO_ARDUINO; 
  return tensao;
}


statesStartStop Motor::ligaMotor(float &veloc){
  if(this->checaEstadoMotor() == DESLIGADO){
    digitalWrite(pinLigaMotor, HIGH);

    const float margemMotorON = 0.2;
    const unsigned long tempoMaxPartida = 4000; // ms
    unsigned long timerPartida = millis();      // ms

    while(millis() - timerPartida <= tempoMaxPartida) {
      calculaVelocidade(veloc);
      float tensao = analisaTensao();
      if(tensao >(tensaoMotorON)){
        return 
      }
    }

    if(checaEstadoMotor() == DESLIGADO ) {
      // FSMState == stateNaoLigou
    }

  }

  digitalWrite(pinLigaMotor, LOW);
  
  this->estadoMotor = checaEstadoMotor() ;
}


void Motor::desligaMotor(float &veloc){
  if(this->checaEstadoMotor() == LIGADO){
    servo.write(0); // posZeroServo

    digitalWrite(pinDesligaMotor, HIGH);

    const unsigned long tempoInjecaoAberta = 4000; // ms
    unsigned long timerInjecaoAberta = millis();      // ms

    while(millis() - timerInjecaoAberta <= tempoInjecaoAberta) {
      // Mantem o rele da Injecao aberto por um tempo
      calculaVelocidade(veloc);
    }

    digitalWrite(pinDesligaMotor, LOW);
    
    this->estadoMotor = checaEstadoMotor();
  }
}


int Motor::desligaStartStop(){
  servo.write(0);
  return 0;
}


bool Motor::checaEstadoMotor() {
  float tensao = analisaTensao();
  if (tensao >= (tensaoMotorON)) { 
    return LIGADO; 
  } else { return DESLIGADO; }
}
