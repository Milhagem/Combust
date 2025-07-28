#include "StartStop.hpp"

int StartStop::tentativasLigar = 0;
int StartStop::tentativasDesligar = 0;  
bool inicioVel = 0;
float tempoInicioVel = 0;
int StartStop::testeBorb=0;
float StartStop::a = pow(2*tempoIncrementoIdealMax - 2*tempoIncrementoIdealMin, 1/aceleraIdeal); 

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
    if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

    // if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
 
    //if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateStart; }  
    
    if (Velocidade::getVelocidade() >= velocidadeMax) {
      return stateStop;
    }

    if ( Velocidade::getAcelera () >= (aceleraIdeal - aceleraIdeal*erroAceitavel)) {
      return stateManipulaBorboleta;
    } else {
      return stateEstabilizaAcelera;
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

    if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

    //if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (Velocidade::getVelocidade() >= velocidadeMax) {
      return stateStop;
    }
    
    if(millis() - tempoUltimoIncremento >= tempoIncrementoIdealMin) {
        if ( millis() - tempoUltimoIncremento >= tempBorbIdeal()) {
            if ( Velocidade::getAcelera () <  aceleraIdeal) {
                motor.incrementaServo();
                tempoUltimoIncremento = millis();
                testeBorb++;
                if (testeBorb==100){return stateSwitchOFF;}
                return stateManipulaBorboleta;
            } else { 
                tempoUltimoIncremento = 0;
                return stateEstabilizaAcelera; 
            }
        } else { return stateManipulaBorboleta; }
    } else { return stateManipulaBorboleta; }
    
}

StartStop::StatesStartStop StartStop::ligaMotorSS (Motor &motor, Display &display) {

    if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

    if ( motor.ligaMotor(display) == Motor::engineON ) {   
      tentativasLigar = 0;    
      return stateStart;
    } else { return stateNotLigou; }
    
}


StartStop::StatesStartStop StartStop::desligaMotorSS (Motor &motor, Display &display) {
    if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

    // if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if ( motor.desligaMotor(display) == Motor::engineOFF ) { 
        tentativasDesligar = 0;      
        return stateStop;
    } else { return stateNotDesligou; }
}

StartStop::StatesStartStop StartStop::start (Motor &motor) {

    if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

    // if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateLigaMotor; }

    if (Velocidade::getVelocidade() < velocidadeMinima) {
      return stateStart;
    } else { return stateEstabilizaAcelera; }

}

StartStop::StatesStartStop StartStop::stop (Motor &motor) {
    if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

    if (motor.checaEstadoMotor() == Motor::engineON) { return stateDesligaMotor; }

    if (Velocidade::getVelocidade() > (velocidadeMinima - velocidadeMinima*erroAceitavel) ) {
        return stateStop;
    } else { return stateStart; }
}


StartStop::StatesStartStop StartStop::freando () {
  if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

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
        tentativasLigar++;
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

float StartStop::tempBorbIdeal() {
   float tempoBorb = pow(a, Velocidade::getAcelera())/2 + tempoIncrementoIdealMin;
   return tempoBorb;
}

