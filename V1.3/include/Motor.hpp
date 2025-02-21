#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"
#include "Velocity.hpp"
#include "Display.hpp"

#define pinLigaMotor    6
#define pinDesligaMotor 5
#define LM2907          A6

#define tensaoMotorON         1.75 
#define TensaoMotorAcelerando 2.75
#define posServoInicial       15

class Motor {
    
public:
    enum statesEngine {engineOFF, engineON, accelerating};

    void setStateEngine ();

    float analisaTensao();

    statesEngine ligaMotor(Display &display);

    statesEngine desligaMotor(Display &display);

    statesEngine checaEstadoMotor();

    void imcrementaServo ();

    void decrementaServo ();


private:
    int posiçãoServo;
    Servo servoMotor;

};

#endif
