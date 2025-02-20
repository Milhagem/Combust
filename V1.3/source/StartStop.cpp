#include "StartStop.hpp"


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

StatesStartStop estabilizaAceleração (Motor &motor) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
 
    if (motor.checaEstadoMotor() == DESLIGADO) { return stateStart; }  
    
    if (Velocidade::getVelocidade() >= (velocidadeMax - velocidadeMax*erroDeAceitaçao) &&  Velocidade::getVelocidade() <= (velocidadeMax + velocidadeMax*erroDeAceitaçao)) {
        return stateEstabilizaVelocidade;
    }

    if ( Velocidade::getAceleração () >= (aceleraçãoIdeal - aceleraçãoIdeal*erroDeAceitaçao) &&  Velocidade::getAceleração () <= (aceleraçãoIdeal + aceleraçãoIdeal*erroDeAceitaçao)) {
        return stateEstabilizaAceleração;
    } else {
        borboleta = mantemAceleração;
        return stateManipulaBorboleta;
    }
    
}

StatesStartStop estabilizaVel (float &tempoInicio) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
 
    if (motor.checaEstadoMotor() == DESLIGADO) { return stateStart; }   
    
    if (millis() - tempoInicio >= tempoMaximoVelocidade) { return stateStop; }

    if (Velocidade::getVelocidade() >= (velocidadeMax - velocidadeMax*erroDeAceitaçao) &&  Velocidade::getVelocidade() <= (velocidadeMax + velocidadeMax*erroDeAceitaçao)) {
        return stateEstabilizaVelocidade;
    } else if (Velocidade::getVelocidade() >= (velocidadeMax + velocidadeMax*erroDeAceitaçao)) {
        borboleta = mantemVelocidadeAcima;
        return stateManipulaBorboleta;
        
    } else if (Velocidade::getVelocidade() <= (velocidadeMax - velocidadeMax*erroDeAceitaçao)) {
        borboleta = mantemVelocideAbaixo;
        return stateManipulaBorboleta;
    }
}

StatesStartStop manipulaBorboleta () {

    if 
    
}

StatesStartStop ligaMotor (Motor &motor) {

    if ( motor.ligaMotor() == engineON ) {
        return stateStart;
    } else { return stateNãoLigou; }
    
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

    if (motor.checaEstadoMotor() == engineOFF) { return stateLigaMotor; }


    if (Velocidade::getVelocidade() < velocidadeMinima) {
        motor.servoWrite(posServoInicial);
        return stateStart;
    } else { return stateEstabilizaAceleração }

}

StatesStartStop stop (Motor &motor) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (motor.checaEstadoMotor() == engineON) { return stateDesligaMotor; }

    if (Velocidade::getVelocidade() > (velocidadeMinima - velocidadeMinima*erroAcel) ) {
        return stateStop;
    } else { return stateStart; }
}

StatesStartStop ligaMotor ();

StatesStartStop freando ();

StatesStartStop nãoLigou (int &tentativas, Display &display) {
    if (tentativas <= 2) {
        return stateLigaMotor;
    } else {
        prin
        return stateDesligaStartStop
    }
}

