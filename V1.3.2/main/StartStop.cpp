#include "StartStop.hpp"

int StartStop::tentativasLigar = 0;
int StartStop::tentativasDesligar = 0;  

int tentativasLigar = 0;
int tentativasDesligar = 0;
bool inicioVel = 0;
float tempoInicioVel = 0;



StartStop::StatesStartStop StartStop::switchOFF () {
    if (digitalRead(switchSS) == LOW) {
        return stateSwitchON;
    } else { return stateSwitchOFF; }
}

StartStop::StatesStartStop StartStop::switchON () {
    if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

    if ( Velocidade::getVelocidade () >= velocidadeMinima ) {
        return stateStop;
    } else if ( Velocidade::getVelocidade ()  >= velZERO ) {
        return stateStart;
    } else { return stateSwitchON; }

} 

StartStop::StatesStartStop StartStop::estabilizaAcelera (Motor &motor) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
 
    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateStart; }  
    
    if (Velocidade::getVelocidade() >= (velocidadeMax - velocidadeMax*erroAceitavel) &&  Velocidade::getVelocidade() <= (velocidadeMax + velocidadeMax*erroAceitavel)) {
        return stateStop;
    }

    if ( Velocidade::getAcelera () >= (aceleraIdeal - aceleraIdeal*erroAceitavel)) {
        return stateEstabilizaAcelera;
    } else {
//       borboleta = manterAcelera;
        return stateManipulaBorboleta;
    }
    
}

//StartStop::StatesStartStop StartStop::estabilizaVelocidade (Motor &motor) {
//    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }
//
//    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
// 
//    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateStart; } 
//    
//    if (inicioVel) { 
//        tempoInicioVel = millis(); 
//        inicioVel = false; 
//    }
//    
//    if (millis() - tempoInicioVel >= tempoMaximoVelocidade) {
//        inicioVel = true;
//        return stateStop; 
//    }
//
//    if (Velocidade::getVelocidade() >= (velocidadeMax - velocidadeMax*erroAceitavel) &&  Velocidade::getVelocidade() <= (velocidadeMax + velocidadeMax*erroAceitavel)) {
//        return stateEstabilizaVelocidade;
//    } else if (Velocidade::getVelocidade() >= (velocidadeMax + velocidadeMax*erroAceitavel)) {
//        borboleta = manterVelocidadeAcima;
//        return stateManipulaBorboleta;
//        
//    } else if (Velocidade::getVelocidade() <= (velocidadeMax - velocidadeMax*erroAceitavel)) {
//        borboleta = manterVelocidadeAbaixo;
//        return stateManipulaBorboleta;
//    }
//}

StartStop::StatesStartStop StartStop::manipulaBorboleta (Motor &motor, float &tempoUltimoIncremento) {

    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if ( millis() - tempoUltimoIncremento >= tempoIncrementoIdeal) {
        if ( Velocidade::getAcelera () <  aceleraIdeal) {
            motor.incrementaServo ();
            tempoUltimoIncremento = millis();
            return stateManipulaBorboleta;
        } else { return stateEstabilizaAcelera; }
    } else { return stateManipulaBorboleta; }
    
}

StartStop::StatesStartStop StartStop::ligaMotorSS (Motor &motor, Display &display) {

    if ( motor.ligaMotor(display) == Motor::engineON ) {   
        tentativasLigar = 0;    
        return stateStart;
    } else { return stateNotLigou; }
    
}


StartStop::StatesStartStop StartStop::desligaMotorSS (Motor &motor, Display &display) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if ( motor.desligaMotor(display) == Motor::engineOFF ) { 
        tentativasDesligar = 0;      
        return stateStop;
    } else { return stateNotDesligou; }

}

StartStop::StatesStartStop StartStop::start (Motor &motor) {

    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateLigaMotor; }


    if (Velocidade::getVelocidade() < velocidadeMinima) {
      return stateStart;
    } else { return stateEstabilizaAcelera; }

}

StartStop::StatesStartStop StartStop::stop (Motor &motor) {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    if (motor.checaEstadoMotor() == Motor::engineON) { return stateDesligaMotor; }

    if (Velocidade::getVelocidade() > (velocidadeMinima - velocidadeMinima*erroAceitavel) ) {
        return stateStop;
    } else { return stateStart; }
}


StartStop::StatesStartStop StartStop::freando () {
    if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

    return stateStop;

}

StartStop::StatesStartStop StartStop::desligaStartStop (Motor& motor, Display &display) {
    if ( motor.desligaMotor(display) == Motor::engineOFF ) {       
        return stateSwitchOFF;
    } else { return stateNotDesligou; }
}

StartStop::StatesStartStop StartStop::notLigou (Display &display) {
    if (tentativasLigar <= 2) {
        tentativasDesligar++;
        delay(1500);
        return stateLigaMotor;
    } else {
        //prin
        return stateDesligaStartStop;
    }
}

StartStop::StatesStartStop StartStop::notDesligou (Display &display) {
    if (tentativasDesligar <= 2) {
        tentativasDesligar++;
        delay(1500);
        return stateDesligaMotor;
    } else {
        //print
        return stateSwitchOFF;
    }
}
