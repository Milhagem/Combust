#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"

#define pinLigaMotor    12
#define pinDesligaMotor 11
#define LM2907          A2 

#define LIGADO         1
#define DESLIGADO      0
#define vecAtual      A9 

#define vecMin         400
#define vecMax         700
#define ZEROvec        150
#define tensaoMotorON 1023/5.0 * 1.2  // 1,6V
class Motor{
  
  private:
    Servo servo;

    bool estadoMotor;

  public:

    void servoAttach(int pin);
    void servoWrite(int value);

    /**
     * Le a tensao
     */
    float analisaTensao();
    
    /**
    * @brief Comuta o rele LIGA MOTOR e isso liga o motor de arranque do carro. Leh a tensao que sai do LM2907 ate que ela seja igual a do motor ligado
    */
    void ligaMotor();
    /**
    * @brief Apos velocidade max ser alcançada, comuta o rele DESLIGA MOTOR e isso desliga o motor do carro. Volta servo pra posicao inicial
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
