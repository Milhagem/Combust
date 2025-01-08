#include "IncrementaVelocidade.h"

//int posServo;                       // graus
//unsigned long timerIncrementoServo = 100; // ms

void giraServoMotor_aceleracao(Motor &motor, int &posServo) {
        if (/*motor.analisaTensao() < TensaoMotorAcelerando && */posServo < posInicialServo) {
            posServo = posInicialServo;
            motor.servoWrite(posServo);
            //timerIncrementoServo = millis();
        } else if(posServo <= 25) {
            posServo += increvementoServo;
            motor.servoWrite(posServo);
            //timerIncrementoServo = millis();
        }
        
        //return posServo;
}

void giraServoMotor_desaceleracao(Motor &motor) {
        motor.servoWrite(0);
        //return posServo;
}