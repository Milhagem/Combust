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
  if(estadoMotor == DESLIGADO){
    digitalWrite(pinLigaMotor, HIGH);
    Serial.println("Ligando Motor");

    for (int i = 0; i < 10; i++) {    
      delay(100);
      float tensao = analisaTensao();
      Serial.print("i: "); Serial.print(i); Serial.println(tensao);
      
      if(tensao > (tensaoMotorON-0.4))
        break;
    }
  }

  digitalWrite(pinLigaMotor,LOW);

  estadoMotor = checaEstadoMotor();
  if(estadoMotor) { Serial.println("Motor ligado!"); }
  else {Serial.println("Motor NAO ligou!"); }
  
  printVelocidade();
}


void Motor::desligaMotor(){
  int pos = 0;
  servo.attach(pos);

  digitalWrite(pinDesligaMotor,HIGH);
  delay(800); // Mantem o rele aberto por um tempo

  digitalWrite(pinDesligaMotor,LOW);
  
  estadoMotor = checaEstadoMotor();
  if(!estadoMotor) { Serial.println("Motor desligado!"); }
  else {Serial.println("Motor NAO desligou!"); }

  printVelocidade();
}


int Motor::desligaStartStop(){
  int pos = 0;
  servo.attach(pos);
  return 0;
}


void Motor::printVelocidade(){
  Serial.print("Velocidade: "); 
  Serial.println(analogRead(vecAtual)); 
}

boolean Motor::checaEstadoMotor() {
  float tensao = analisaTensao();
  if (tensao > (tensaoMotorON-0.2)) { return LIGADO; }
  else { return DESLIGADO; }
}