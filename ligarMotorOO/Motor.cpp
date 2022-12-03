#include "Motor.h"

void Motor::servoAttach(int pin){
  this->servo.attach(pin);
}


void Motor::servoWrite(int value){
  this->servo.write(value);
}


float Motor::analisaTensao(){
  float valorInicial = analogRead(A1);
  float tensao = (valorInicial*5.0) / 1024; 
  return tensao;
}


void Motor::ligaMotor(){
  digitalWrite(pLigaMotor, HIGH);
  Serial.println("Ligando Motor");
  // Verifica a tensao por 1s
  for (int i = 0; i < 10; i++) {    
    delay(100);
    float tensao = analisaTensao();
    if(tensao < (tensaoLigado+1) && tensao > (tensaoLigado-1)) { i++; } 
    else { i = 0; }
  }

  digitalWrite(pLigaMotor,LOW);  // liga motor de arranque por 800ms ??????
  estadoMotor = LIGADO;
  digitalWrite(9, HIGH);

  Serial.println("Motor ligado");
  printVelocidade();
}


void Motor::desligaMotor(){
  int pos = 0;
  servo.attach(pos);

  digitalWrite(pDesligaMotor,HIGH);
  digitalWrite(9,LOW);
  delay(800);

  digitalWrite(pDesligaMotor,LOW);
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