#include "Aceleracao.h"
#include "IncrementaVelocidade.h"

float aceleracao_otima = 1;

void manipula_aceleracao_motor(float &aceleracao, int &posServo, unsigned long &time_variacao, Motor &motor){
    if (aceleracao <= aceleracao_otima + 0.5){
        if(time_variacao < 50){
            time_variacao = time_variacao;
        }
        time_variacao -= 2;
        giraServoMotor_aceleracao(motor,posServo);
    }
    if (aceleracao >= aceleracao_otima - 0.5){
        if(time_variacao > 50){
            time_variacao = time_variacao;
        }
        time_variacao += 2;
        giraServoMotor_aceleracao(motor,posServo); 
    }
}