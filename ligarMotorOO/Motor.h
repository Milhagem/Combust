#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"

#define pinLigaMotor    13 // CONFERIR
#define pinDesligaMotor 12 // CONFERIR

#define LIGADO         1
#define DESLIGADO      0
#define vecAtual      A0 // CONFERIR
#define comparaTensao A1 // CONFERIR

#define vecMin       100 // CONFERIR
#define vecMax       300 // CONFERIR
#define ZEROvec       50 // CONFERIR
#define tensaoMotorON  5 // CONFERIR

class Motor{
  
  private:
    Servo servo;

    bool estadoMotor;

  public:

    void servoAttach(int pin);
    void servoWrite(int value);

    float analisaTensao();
    void ligaMotor();
    /**
    * @brief Apos velocidade max ser alcançada, desliga motor e volta servo pra posicao inicial
    *
    */
    void desligaMotor();
    void printVelocidade();
    /**
     * @brief desliga o Start Stop (passa para o Modo Manual) e volta o servo para a posicao inicial
     */
    int desligaStartStop();



};

#endif