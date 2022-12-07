#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"

#define pinLigaMotor    12 // CONFERIDO
#define pinDesligaMotor 11 // CONFERIDO

#define LIGADO         1
#define DESLIGADO      0
#define vecAtual      A9 
#define comparaTensao A2

#define vecMin         400      // CONFERIR
#define vecMax         700      // CONFERIR
#define ZEROvec         50      // CONFERIR
#define tensaoMotorON 1024/5    // CONFERIR (proporção estimada: quando está ligado, 1V (de um máximo de 5V))

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
