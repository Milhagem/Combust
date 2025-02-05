#include "StartStop.hpp"

StartStop () : FSMstate(stateSwitchOFF){}

StatesStartStop getFSMstate() { return FSMstate; }

void setFSMstate(StatesStartStop state) { FSMstate = state; }

StatesStartStop switchOFF () {
    float velocidade = calculateSpeed();
    if(digitalRead(switchSS) == PRESSIONADO){
        return stateSwitchON;
    }
    return stateSwitchOFF;
}

StatesStartStop switchON () {
    float velocidade = calculateSpeed();
    if(digitalRead(switchSS) == NOT_PRESSIONADO){
        return desligaMotor();
    }

    if(velocidade >= velZERO){
        return monitoraVel();
    }
    return stateSwitchON;
} 

StatesStartStop monitoraVel () {
    float velocidade = calculateSpeed();
    if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        return desligaMotor();
    } 

    if(digitalRead(pinFreio) == PRESSIONADO) {
        return freando();
    }

    if(velocidade<velMin && velocidade>=velZERO) {
        if(motor.checaEstadoMotor() == DESLIGADO) {
            return ligaMotor();
        } else if(motor.checaEstadoMotor() == LIGADO) { 
            return incrementaVel();
        }
    }

    if(velocidade > velMax && motor.checaEstadoMotor() == LIGADO) {
        return desligaMotor();
    } 
    return monitoraVel();
}

StatesStartStop incrementaVel () {
    float velocidade = calculateSpeed();
    if(velocidade>=velMax) {
        return estabilizaVel();
    }
    return incrementaVel();
}

StatesStartStop desligaMotor () {
    float velocidade = calculateSpeed();
    if(digitalRead(switchSS) == NOT_PRESSIONADO) {
        return desligaMotor();
    }

    if(digitalRead(pinFreio) == PRESSIONADO) {
        return freando();
    }

    motor.ligaMotor(velocidade);
    if (motor.estadoMotor == TRUE){
        return monitoraVel();
    }
    else{
        return nãoLigou();
    }
    return desligaMotor();
}

StatesStartStop ligaMotor ();

StatesStartStop freando ();

StatesStartStop nãoLigou ();

StatesStartStop estabilizaVel ();
