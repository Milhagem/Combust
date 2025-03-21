#include "include/Motor.hpp"
#include "include/Display.hpp"
#include "include/Velocidade.hpp"


Motor::Motor () : posServo(0) {
  servo.attach(pinServo);
  servo.write(0);
}

float Motor::analisaTensao(){
  const float RESOLUCAO_ARDUINO = 1024.0;
  const float TENSAO_ARDUINO = 5.0;

  float valorInicial = analogRead(LM2907); 
  float tensao = (valorInicial/RESOLUCAO_ARDUINO)*TENSAO_ARDUINO; 
  return tensao;
}


Motor::statesEngine Motor::ligaMotor(Display& display){
  if (this->checaEstadoMotor() == engineOFF) {
    const float margemMotorON = 0.2;
    const unsigned long tempoMaxPartida = 4000; // ms
    unsigned long timerPartida = millis();      // ms

    servo.write(posServoInicial);
    posServo = posServoInicial;
    digitalWrite(pinLigaMotor, HIGH);


    while(millis() - timerPartida <= tempoMaxPartida) {
      display.mostraTensaoEVel(Velocidade::calculaVelocidade());
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


Motor::statesEngine Motor::desligaMotor(Display& display){
  if(this->checaEstadoMotor() == engineON){
    servo.write(0); // posZeroServo
    posServo = 0;

    digitalWrite(pinDesligaMotor, HIGH);

    const unsigned long tempoInjecaoAberta = 4000; // ms
    unsigned long timerInjecaoAberta = millis();      // ms

    while(millis() - timerInjecaoAberta <= tempoInjecaoAberta) {
      // Mantem o rele da Injecao aberto por um tempo
      display.mostraTensaoEVel(Velocidade::calculaVelocidade());
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


Motor::statesEngine Motor::checaEstadoMotor() {
  float tensao = analisaTensao();
  if (tensao >= (tensaoMotorON)) { 
    return engineON; 
  } else { return engineOFF; }
}

void Motor::incrementaServo () {
  posServo += 1;
  servo.write(posServo);

}

void Motor::decrementaServo () {
  posServo -= 1;
  servo.write(posServo);
}
