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

    void servoAttach(int pin);
<<<<<<< HEAD
      
=======

>>>>>>> bafe57cb0758a4fdf6eac0270abd71f2f858d5c7
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
