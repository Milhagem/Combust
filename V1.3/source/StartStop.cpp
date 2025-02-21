#include "StartStop.hpp"


StatesStartStop StartStop::switchOFF () {
    if (digitalRead(switchSS) == PRESSIONADO) {
        return stateSwitchON;
    } else { return stateSwitchOFF; }
}

StatesStartStop StartStop::switchON () {
    if (digitalRead(switchSS) == NOT_PRESSIONADO){ return stateDesligaStartStop; }

    if ( Velocidade::getVelocidade () >= velocidadeMinima ) {
        return stateStop;
    } else if ( Velocidade::getVelocidade ()  >= velZERO ) {
        return stateStart;
    } else { return stateSwitchON; }

} 

StatesStartStop StartStop::estabilizaAceleração (Motor &motor) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
 
    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateStart; }  
    
    if (Velocidade::getVelocidade() >= (velocidadeMax - velocidadeMax*erroDeAceitaçao) &&  Velocidade::getVelocidade() <= (velocidadeMax + velocidadeMax*erroDeAceitaçao)) {
        return stateEstabilizaVelocidade;
    }

    if ( Velocidade::getAceleração () >= (aceleraçãoIdeal - aceleraçãoIdeal*erroDeAceitaçao) &&  Velocidade::getAceleração () <= (aceleraçãoIdeal + aceleraçãoIdeal*erroDeAceitaçao)) {
        return stateEstabilizaAceleração;
    } else {
        borboleta = manterAceleração;
        return stateManipulaBorboleta;
    }
    
}

StatesStartStop StartStop::estabilizaVelocidade (Motor &motor, float &tempoInicio) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
 
    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateStart; }   
    
    if (millis() - tempoInicio >= tempoMaximoVelocidade) { return stateStop; }

    if (Velocidade::getVelocidade() >= (velocidadeMax - velocidadeMax*erroDeAceitaçao) &&  Velocidade::getVelocidade() <= (velocidadeMax + velocidadeMax*erroDeAceitaçao)) {
        return stateEstabilizaVelocidade;
    } else if (Velocidade::getVelocidade() >= (velocidadeMax + velocidadeMax*erroDeAceitaçao)) {
        borboleta = manterVelocidadeAcima;
        return stateManipulaBorboleta;
        
    } else if (Velocidade::getVelocidade() <= (velocidadeMax - velocidadeMax*erroDeAceitaçao)) {
        borboleta = manterVelocidadeAbaixo;
        return stateManipulaBorboleta;
    }
}

StatesStartStop StartStop::manipulaBorboleta (Motor &motor, float &tempoUltimoImcremento) {

    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if ( millis() - tempoUltimoImcremento >= tempoImcrementoIdeal) {
        if (borboleta == manterAceleração) {
            motor.imcrementaServo ();
            return stateEstabilizaAceleração;
        } else if (borboleta == manterVelocidadeAcima) {
            motor.decrementaServo ();
            return stateEstabilizaVelocidade;
        } else if (borboleta == manterVelocidadeAbaixo) {
            motor.imcrementaServo ();
            return stateEstabilizaVelocidade;
        }
    } else {
        if (borboleta == manterAceleração) {
            return stateEstabilizaAceleração;
        } else if (borboleta == manterVelocidadeAcima || borboleta == manterVelocidadeAbaixo) {
            return stateEstabilizaVelocidade;
        }

    }
    
}

StatesStartStop StartStop::ligaMotor (Motor &motor, Display &display) {

    if ( motor.ligaMotor(display) == Motor::engineON ) {       
        return stateStart;
    } else { return stateNãoLigou; }
    
}


StatesStartStop StartStop::desligaMotor (Motor &motor, Display &display) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if ( motor.desligaMotor(display) == Motor::engineOFF ) {       
        return stateStop;
    } else { return stateNãoDesligou; }

}

StatesStartStop StartStop::start (Motor &motor) {

    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateLigaMotor; }


    if (Velocidade::getVelocidade() < velocidadeMinima) {
        motor.servoWrite(posServoInicial);
        return stateStart;
    } else { return stateEstabilizaAceleração }

}

StatesStartStop StartStop::stop (Motor &motor) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (motor.checaEstadoMotor() == Motor::engineON) { return stateDesligaMotor; }

    if (Velocidade::getVelocidade() > (velocidadeMinima - velocidadeMinima*erroDeAceitaçao) ) {
        return stateStop;
    } else { return stateStart; }
}


StatesStartStop StartStop::freando () {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    return stateStop;

}

StatesStartStop StartStop::desligaStartStop (Display &display) {
    if ( motor.desligaMotor(display) == Motor::engineOFF ) {       
        return stateSwitchOFF;
    } else { return stateNãoDesligou; }
}

StatesStartStop StartStop::nãoLigou (int &tentativas, Display &display) {
    if (tentativas <= 2) {
        return stateLigaMotor;
    } else {
        //prin
        return stateDesligaStartStop;
    }
}

SatesStartStop StartStop::nãoDesligou (int &tentativas, Display &display) {
    if (tentativas <= 2) {
        return stateDesligaMotor;
    } else {
        //print
        return stateSwitchOFF;
    }
}
