//parei em começar atualizar o startstop .cpp


#include "StartStop.hpp"

int StartStop::tentativasLigar = 0;
int StartStop::tentativasDesligar = 0;  
bool StartStop::inicioVel = 0;
float StartStop::tempoInicioVel = 0.0f;
int StartStop::testeBorb = 0;
int StartStop::modoControle = 0;
unsigned long StartStop::timerTentativa = 0;

void StartStop::Inicializar_sensores_startstop(){
    pinMode(pinFreio, INPUT_PULLUP);
    pinMode(switchSS, INPUT_PULLUP);
}

StartStop::StatesStartStop StartStop::switchOFF () {
    testeBorb = 0;

    if (digitalRead(switchSS) == LOW) {
        return stateSwitchON;
    } else { return stateSwitchOFF; }
}

StartStop::StatesStartStop StartStop::switchON () {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    if (Velocidade::getVelocidade() >= velocidadeMinima) {
        return stateStop;
    } else { return stateStart; } 
}

StartStop::StatesStartStop StartStop::desligaStartStop (Motor& motor, Display &display, Sensores_motor &sensores) {
    if (motor.desligaMotor(display, sensores) == Sensores_motor::engineOFF) {       
        return stateSwitchOFF;
    } else { return stateNotDesligou; }
}

StartStop::StatesStartStop StartStop::start (Motor &motor, Sensores_motor &sensores) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    // if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (sensores.analisa_status_motor() == Sensores_motor::engineOFF) { return stateLigaMotor; }

    if (Velocidade::getVelocidade() < velocidadeMinima) {
        return stateStart;
    } else { return stateEstabilizaAcelera; }
}

StartStop::StatesStartStop StartStop::stop (Motor &motor, Sensores_motor &sensores) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    if (sensores.analisa_status_motor() == Sensores_motor::engineON) { return stateDesligaMotor; }

    if (Velocidade::getVelocidade() > (velocidadeMinima - velocidadeMinima*erroAceitavel)) {
        return stateStop;
    } else { return stateStart; }
}

StartStop::StatesStartStop StartStop::estabilizaAcelera (Motor &motor, Sensores_motor &sensores) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    // if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
 
    if (sensores.analisa_status_motor() == Sensores_motor::engineOFF) { return stateStart; }

    if (Velocidade::getVelocidade() >= velocidadeMax) {
        return stateStop;
    }

     if(modoControle == 0){
    BSFC::Controle_RPM(RPMideal, sensores.getRpm(), sensores.analisa_status_motor(), sensores.status_central);
    } else if (modoControle == 1){
    BSFC::Controle_TPS(PosBorboIdeal, sensores.getPosBorbo(), sensores.analisa_status_motor(), sensores.status_central);
    } 

      return stateEstabilizaAcelera;  
}


StartStop::StatesStartStop StartStop::ligaMotorSS (Motor &motor, Display &display, Sensores_motor &sensores) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    if (motor.ligaMotor(display, sensores) == Sensores_motor::engineON ) {   
        tentativasLigar = 0;    
        return stateStart;
    } else { 
        timerTentativa = millis(); 
        return stateNotLigou; 
    }
}

StartStop::StatesStartStop StartStop::desligaMotorSS (Motor &motor, Display &display, Sensores_motor &sensores) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    // if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (motor.desligaMotor(display,sensores) == Sensores_motor::engineOFF) { 
        tentativasDesligar = 0;      
        return stateStop;
    } else { return stateNotDesligou; }
}

StartStop::StatesStartStop StartStop::notLigou (Display &display) {
    if (tentativasLigar <= 2) {
        if (millis() - timerTentativa >= 1500) {
            tentativasLigar++;
            return stateLigaMotor;
        }
        return stateNotLigou;
    } else { return stateDesligaStartStop; }
}

StartStop::StatesStartStop StartStop::notDesligou (Display &display) {
    if (tentativasDesligar <= 2) {
        if (millis() - timerTentativa >= 1500) {
            tentativasDesligar++;
            return stateDesligaMotor;
        }
        return stateNotDesligou;
    } else { return stateSwitchOFF; }
}

StartStop::StatesStartStop StartStop::freando () {
  if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

  if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

  return stateStop;
}
