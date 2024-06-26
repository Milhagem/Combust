#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"

#define pinLigaMotor    6
#define pinDesligaMotor 5
#define LM2907          A2 

#define LIGADO    1
#define DESLIGADO 0
#define velAtual  A10 



#define VelMin         400  // Valores para teste
#define VelMax         700  // Valores para teste
#define ZEROVel        150  // Valores para teste
#define tensaoMotorON  2.75

class Motor{
  
  private:
    Servo servo;
    bool estadoMotor;

  public:

    void servoAttach(int pin);
    void servoWrite(int value);

    /**
     * Leh a tensao de saida do LM2907
     * @return tensao entre 0V e 5V
     */
    float analisaTensao();
    /**
    * @brief Fecha o rele LIGA MOTOR e isso liga o motor de arranque por no maximo 2 segundos
    */
    void ligaMotor();
    /**
    * @brief Abre o rele DESLIGA MOTOR e isso desliga o motor do carro. Volta servo pra posicao inicial
    *
    */
    void desligaMotor();
    /**
     * @brief desliga o Start Stop (passa para o Modo Manual) e volta o servo para a posicao inicial
     */
    int desligaStartStop();
    /**
     * @return LIGADO (expands to 1) ou DESLIGADO (expands to 0)
    */
    boolean checaEstadoMotor();

    bool getEstadoMotor() { return this->estadoMotor; }
    void setEstadoMotor(bool estadoMotor) { this->estadoMotor = estadoMotor; }



};

#endif
