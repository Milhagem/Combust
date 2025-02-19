#include "StartStop.hpp"

StartStop () : FSMstate(stateSwitchOFF) {}

static float getVelocidadeMaxVariavel () { return velocidadeMaxVariavel; }

static void setVelocidadeMaxVariavel (float &velMax) { velocidadeMaxVariavel = velMax; }

StatesStartStop switchOFF () {
    if (digitalRead(switchSS) == PRESSIONADO) {
        return stateSwitchON;
    } else { return stateSwitchOFF; }
}

StatesStartStop switchON () {
    if (digitalRead(switchSS) == NOT_PRESSIONADO){
        return stateDesligaStartStop;
    }

    if ( Velocidade::getVelocidade () >= velocidadeMinima ) {
        return stateStop;
    } else if ( Velocidade::getVelocidade ()  >= velZERO ) {
        return stateStart;
    } else { return stateSwitchON; }

} 

StatesStartStop estabilizaAceleração () {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

   
    if(motor.checaEstadoMotor() == DESLIGADO) {
        return stateLigaMotor;
    }    
    
    if ( Velocidade::getVelocidade () >= velocidadeMaxVariavel ) {
        velocidadeMaxVariavel = Velocidade::getVelocidade() + 2;
        tempoIncrementoBorboleta += 50;
        return stateManipulaBorboleta;
    } else { return stateManipulaBorboleta; }
    
}

StatesStartStop estabilizaVel ();

StatesStartStop manipulaBorboleta () {
    
}

StatesStartStop ligaMotor () {
    
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

StatesStartStop start (Motor &motor) {

    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (motor.checaEstadoMotor() == engineOFF) { return stateLigaMotor }


    if (Velocidade::getVelocidade() < velocidadeMinima) {
        motor.servoWrite(posServoInicial);
        return stateStart;
    } else { return stateEstabilizaAceleração }

}

StatesStartStop stop ()

StatesStartStop ligaMotor ();

StatesStartStop freando ();

StatesStartStop nãoLigou ();

