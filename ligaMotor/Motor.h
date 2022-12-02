#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "Servo.h"

#define LIGADO 1
#define DESLIGADO 0

class Motor{
  private:
    Servo servo;
    int pLigaMotor;
    int pDesligaMotor;
    bool estadoMotor;
    float tensaoLigado;

  public:
    float analisaTensao();
    void ligaMotor();
    /**
    * @brief Apos velocidade max ser alcançada, desliga motor e volta servo pra posicao inicial
    *
    */
    void desligaMotor();
    void printVelocidade();

};

#endif