#include "StartStop.hpp"

int StartStop::tentativasLigar = 0;
int StartStop::tentativasDesligar = 0;  

int tentativasLigar = 0;
int tentativasDesligar = 0;
bool inicioVel = 0;
float tempoInicioVel = 0;


StartStop::StatesStartStop StartStop::switchOFF () {
    //delay(4000);
    if (digitalRead(switchSS) == LOW) {
        return stateSwitchON;
    } else { return stateSwitchOFF;}
}

StartStop::StatesStartStop StartStop::switchON () {
    if (digitalRead(switchSS) == HIGH){ return stateDesligaStartStop; }

    return stateStart;

} 

StartStop::StatesStartStop StartStop::estabilizaAcelera (Motor &motor) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }
    //if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateStart;}  
    

    // Barramente Segurança
    /*if (Velocidade::getVelocidade() >= (velocidadeMax) - velocidadeMax*erroAceitavel){
        return stateStop;
    }*/

    return stateManipulaBorboleta;
}

StartStop::StatesStartStop StartStop::manipulaBorboleta (Motor &motor, float &tempoUltimoIncremento) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }
    //if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

 
    if (Velocidade::getVelocidade() >= velocidadeMax){
        //Uma chamada para a função de manipulação de parametros pode ser colocada aqui.
        return stateStop;
    }

    if( millis() - tempoUltimoIncremento >= tempoIncrementoIdeal) {
        // Incrementa o servo para a aceleração abaixo do ideal + Barreira segurança velocidade maxima
        if (Velocidade::getAcelera() < aceleraIdeal/* && Velocidade::getVelocidade() <= velocidadeMax*/) {
            motor.incrementaServo();
            tempoUltimoIncremento = millis();
        }
        if (Velocidade::getAcelera() >= aceleraIdeal /*&& Velocidade::getVelocidade() <= velocidadeMax*/) {
            // Se a aceleração for maior que o ideal, decrementa o servo
            motor.decrementaServo();
            tempoUltimoIncremento = millis();
        }
        return stateManipulaBorboleta;
    }

}


StartStop::StatesStartStop StartStop::ligaMotorSS (Motor &motor, Display &display) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop;}
    if (motor.checaEstadoMotor() == Motor::engineOFF){
        if ( motor.ligaMotor(display) == Motor::engineON ) {   
            //tentativasLigar = 0;    
            return stateStart;
        } else { return stateNotLigou; }
    }else{
        return stateStart;
    }
}

StartStop::StatesStartStop StartStop::desligaMotorSS (Motor &motor, Display &display) {
    if (digitalRead(switchSS) == HIGH)
     { return stateDesligaStartStop; }
    //if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
    
    if ( motor.desligaMotor(display) == Motor::engineOFF ) { 
        //tentativasDesligar = 0;      
        return stateStop;
    } else { return stateNotDesligou; }


}

StartStop::StatesStartStop StartStop::start (Motor &motor, float &tempoIncrementoInicial) {

    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    //if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (motor.checaEstadoMotor() == Motor::engineOFF) { return stateLigaMotor; }

    // estrutura de incremento inicial para testes --------------------------------
    if ( millis() - tempoIncrementoInicial >= 1) {
        if(Velocidade::getVelocidade() >= 0){
            motor.incrementaServo();
            tempoIncrementoInicial = millis();
        }
        return stateStart;
    }
    // Estrategia alternativa (Ajuste manual de acordo com a posição da peça do servo)
    //motor.servoWrite(1190);

    return stateEstabilizaAcelera;
}

StartStop::StatesStartStop StartStop::stop (Motor &motor) {
    //motor.servoWrite(800);
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    if (motor.checaEstadoMotor() == Motor::engineON) { return stateDesligaMotor; }

    if (Velocidade::getVelocidade() >= velocidadeMinima){ return stateStop; }

    return stateSwitchOFF;
}


StartStop::StatesStartStop StartStop::freando () {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }
    return stateStop;
}

StartStop::StatesStartStop StartStop::desligaStartStop (Motor& motor, Display &display) {
    if ( motor.desligaMotor(display) == Motor::engineOFF ) {       
        return stateNotDesligou;
    }else { return stateSwitchOFF;}
}

StartStop::StatesStartStop StartStop::notLigou (Display &display) {
    delay(3000);
    if (digitalRead(switchSS) == HIGH) {return stateSwitchOFF;}
    return stateLigaMotor;
}

StartStop::StatesStartStop StartStop::notDesligou (Display &display) {
    delay(3000);
    if (digitalRead(switchSS) == HIGH) {return stateSwitchOFF;}
    //return stateNotDesligou;
    return stateSwitchOFF;
}

StartStop::StatesStartStop StartStop::manipulaParametros(Display &display) {
    
    return ;
}
