#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"

#define LIGADO 1
#define DESLIGADO 0
#define vecAtual      A0
#define comparaTensao A1

#define vecMin       100
#define vecMax       300
#define ZEROvec       50
#define tensaoMotorON  5

class Motor{
  private:
    int pLigaMotor;
    int pDesligaMotor;
    bool estadoMotor;
    float tensaoLigado;

  public:
    Servo servo;
    float analisaTensao();
    void ligaMotor();
    /**
    * @brief Apos velocidade max ser alcançada, desliga motor e volta servo pra posicao inicial
    *
    */
    void desligaMotor();
    void printVelocidade();
    int desligaStartStop();

};

#endif