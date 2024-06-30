#include "IncrementaVelocidade.h"

int posServo;                       // graus
unsigned int timerIncrementoServo; // ms

void giraServoMotor(Motor &motor) {
    if(millis() - timerIncrementoServo > intervIncrementaVel) {
        if (motor.analisaTensao() < TensaoMotorAcelerando && posServo <= posInicialServo) {
            posServo = posInicialServo;
            motor.servoWrite(posServo);
            timerIncrementoServo = millis();
        } else {
            posServo += increvementoServo;
            motor.servoWrite(posServo);
            timerIncrementoServo = millis();
        }
    }
}