#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"
#include "Velocidade.h"

#define pinLigaMotor    6
#define pinDesligaMotor 5
#define LM2907          A6

#define tensaoMotorON         1.75 
#define TensaoMotorAcelerando 2.75

class Motor {
    
public:
    enum statesEngine {engineOFF, engineON, accelerating};

    void setStateEngine ();

    statesEngine Motor::checaEstadoMotor ();

    float Motor::analisaTensao();

    void Motor::ligaMotor(float &veloc);

    void Motor::desligaMotor(float &veloc);

    bool Motor::desligaStartStop();

    statesEngine Motor::checaEstadoMotor();


private:
    statesEngine stateEngine;

};

#endif
