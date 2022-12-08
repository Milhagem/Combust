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
    Serial.println("Ligando Motor");

    for (int i = 0; i < 20; i++) {    
      delay(100);
      float tensao = analisaTensao();      
      if(tensao > (tensaoMotorON-0.4))
        break;
    }
  }

  digitalWrite(pinLigaMotor,LOW);

  estadoMotor = checaEstadoMotor();
}


void Motor::desligaMotor(){
  if(this->checaEstadoMotor() == LIGADO){
    int pos = 0;
    servo.attach(pos);

    digitalWrite(pinDesligaMotor,HIGH);
    delay(800); // Mantem o rele aberto por um tempo

    digitalWrite(pinDesligaMotor,LOW);
    
    this->estadoMotor = checaEstadoMotor();
  }
}


int Motor::desligaStartStop(){
  int pos = 0;
  servo.attach(pos);
  return 0;
}


boolean Motor::checaEstadoMotor() {
  float tensao = analisaTensao();
  if (tensao > (tensaoMotorON-0.2)) { return LIGADO; }
  else { return DESLIGADO; }
}