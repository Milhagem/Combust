#include "Motor.hpp"

Motor () posiçaoServo(0) {

  servoAttach(pinServo);
  servoWrite(0);
  
}




float Motor::analisaTensao(){
  const float RESOLUCAO_ARDUINO = 1024.0;
  const float TENSAO_ARDUINO = 5.0;

  float valorInicial = analogRead(LM2907); 
  float tensao = (valorInicial/RESOLUCAO_ARDUINO)*TENSAO_ARDUINO; 
  return tensao;
}


statesEngine Motor::ligaMotor(Display &display){
  if (this->checaEstadoMotor() == engineOFF) {
    const float margemMotorON = 0.2;
    const unsigned long tempoMaxPartida = 4000; // ms
    unsigned long timerPartida = millis();      // ms

    servoMotor.write(posServoInicial);
    posiçãoServo = posServoInicial;
    digitalWrite(pinLigaMotor, HIGH);


    while(millis() - timerPartida <= tempoMaxPartida) {
      Velocidade::calculaVelocidade();
      display.mostraTensaoEVel(*this, Velocidade::getVelocidade());
      float tensao = analisaTensao();
      if (tensao > tensaoMotorON) {
        digitalWrite(pinLigaMotor, LOW);
        return engineON;
      }
    }

    if(checaEstadoMotor() == engineOFF ) {
      digitalWrite(pinLigaMotor, LOW);
      return engineOFF;
    } else {
      digitalWrite(pinLigaMotor, LOW); 
      return engineON;
    }

  }

}


statesEngine Motor::desligaMotor(Display &display){
  if(this->checaEstadoMotor() == engineON){
    servo.write(0); // posZeroServo
    posiçãoServo = 0;

    digitalWrite(pinDesligaMotor, HIGH);

    const unsigned long tempoInjecaoAberta = 4000; // ms
    unsigned long timerInjecaoAberta = millis();      // ms

    while(millis() - timerInjecaoAberta <= tempoInjecaoAberta) {
      // Mantem o rele da Injecao aberto por um tempo
      Velocidade::calculaVelocidade();
      display.mostraTensaoEVel(*this, Velocidade::getVelocidade());
      if (checaEstadoMotor() == engineOFF) {
        digitalWrite(pinDesligaMotor, LOW);
        return engineOFF;
      }      
    }

    if( checaEstadoMotor() == engineOFF ) {
      digitalWrite(pinDesligaMotor, LOW);
      return engineOFF;
    } else {
      digitalWrite(pinDesligaMotor, LOW); 
      return engineON;
    }    
  }
}


statesEngine Motor::checaEstadoMotor() {
  float tensao = analisaTensao();
  if (tensao >= (tensaoMotorON)) { 
    return engineON; 
  } else { return engineOFF; }
}

void Motor::imcrementaServo () {
  posiçãoServo += 1;
  servoMotor.write(posiçãoServo);

}

void Motor::decrementaServo () {
  posiçãoServo -= 1;
  servoMotor.write(posiçãoServo);
}
