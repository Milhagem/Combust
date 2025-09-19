#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "Servo.h"
#include "Velocidade.hpp"
#include "Display.hpp"

class Display;

#define pinLigaMotor    6
#define pinDesligaMotor 5
#define LM2907          A2
#define pinServo        8

#define tensaoMotorON         0.8
#define TensaoMotorAcelerando 2.75
#define posServoInicial       20

class Motor {
    
public:
    enum statesEngine {engineOFF, engineON, accelerating};

    void servoAttach(int pin);
    
    void servoWrite(int value);

    float analisaTensao();

    statesEngine ligaMotor(Display& display);

    statesEngine desligaMotor(Display& display);

    statesEngine checaEstadoMotor();

    void incrementaServo ();

    void decrementaServo ();

    void zeraServo (int valor);


private:
    int posServo = 0;
    Servo servo;

};

#endif
