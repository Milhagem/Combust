#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"
#include "Velocidade.hpp"
#include "Display.hpp"

class Display;

#define pinLigaMotor    6
#define pinDesligaMotor 5
#define LM2907          A6
#define pinServo        8

#define tensaoMotorON         1.75 
#define TensaoMotorAcelerando 2.75
#define posServoInicial       15

class Motor {
    
public:
    enum statesEngine {engineOFF, engineON, accelerating};

    Motor ();

    float analisaTensao();

    statesEngine ligaMotor(Display& display);

    statesEngine desligaMotor(Display& display);

    statesEngine checaEstadoMotor();

    void incrementaServo ();

    void decrementaServo ();


private:
    int posServo;
    Servo servo;

};

#endif
