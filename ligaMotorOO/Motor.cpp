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
  digitalWrite(pinLigaMotor, HIGH);
  Serial.println("Ligando Motor");
  for (int i = 0; i < 10; i++) {    
    delay(100);
    float tensao = analisaTensao();
    Serial.print("i: ");
    Serial.print(i);
    Serial.println(tensao);
    if(tensao > (tensaoMotorON-0.4)) { i++; } 
    else { i = 0; }
  }

  digitalWrite(pinLigaMotor,LOW);
  estadoMotor = LIGADO;
  digitalWrite(9, HIGH); // Por que isso?

  Serial.println("Motor ligado");
  printVelocidade();
}


void Motor::desligaMotor(){
  int pos = 0;
  servo.attach(pos);

  digitalWrite(pinDesligaMotor,HIGH);
  // digitalWrite(9,LOW); // Por que isso?
  delay(800);

  digitalWrite(pinDesligaMotor,LOW);
  estadoMotor = DESLIGADO;

  Serial.println("Motor desligado");
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
