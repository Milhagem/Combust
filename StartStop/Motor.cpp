#include "Motor.h"

void Motor::servoAttach(int pin){
  this->servo.attach(pin);
}


void Motor::servoWrite(int value){
  this->servo.write(value);
}


float Motor::analisaTensao(){
  float valorInicial = analogRead(LM2907); 
  float tensao = (valorInicial/1024.0)*5.0; 
  return tensao;
}


void Motor::ligaMotor(){
  if(this->checaEstadoMotor() == DESLIGADO){
    digitalWrite(pinLigaMotor, HIGH);

    for (int i = 0; i < 10; i++) {    
      delay(100);
      float tensao = analisaTensao();      
      if(tensao > (tensaoMotorON-0.2))
        break;
    }
  }

  digitalWrite(pinLigaMotor, LOW);
  
  this->estadoMotor = checaEstadoMotor();
}


void Motor::desligaMotor(){
  if(this->checaEstadoMotor() == LIGADO){
    servo.write(0);

    digitalWrite(pinDesligaMotor, HIGH);
    delay(1600); // Mantem o rele aberto por um tempo

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