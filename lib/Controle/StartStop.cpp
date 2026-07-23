
#include "StartStop.hpp"

int StartStop::tentativasLigar = 0;
int StartStop::tentativasDesligar = 0;  
bool StartStop::inicioVel = 0;
float StartStop::tempoInicioVel = 0.0f;
int StartStop::testeBorb = 0;
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

StartStop::StatesStartStop StartStop::desligaStartStop (Motor& motor, Display &display) {
    if (motor.desligaMotor(display) == Motor::engineOFF) {       
        return stateSwitchOFF;
    } else { return stateNotDesligou; }
}

StartStop::StatesStartStop StartStop::start (Motor &motor) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    // if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }

    if (motor.analisa_status_motor() == Motor::engineOFF) { return stateLigaMotor; }

    if (Velocidade::getVelocidade() < velocidadeMinima) {
        return stateStart;
    } else { return stateEstabilizaAcelera; }
}

StartStop::StatesStartStop StartStop::stop (Motor &motor) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    if (motor.analisa_status_motor() == Motor::engineON) { return stateDesligaMotor; }

    if (Velocidade::getVelocidade() > (velocidadeMinima - velocidadeMinima*erroAceitavel)) {
        return stateStop;
    } else { return stateStart; }
}

StartStop::StatesStartStop StartStop::estabilizaAcelera (Motor &motor) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    // if (digitalRead(pinFreio) == PRESSIONADO) { return stateFreando; }
 
    if (motor.analisa_status_motor() == Motor::engineOFF) { return stateStart; }

    if (Velocidade::getVelocidade() >= velocidadeMax) {
        return stateStop;
    }

   if (modoControle == 1) {
    BSFC::Controle_RPM(RPMideal, Ckp::getRpm(), motor.analisa_status_motor(), Motor::getStatusCentral());
} else if (modoControle == 0) {
    BSFC::Controle_TPS(PosBorboIdeal, TPS::getPosBorbo(), motor.analisa_status_motor(), Motor::getStatusCentral());
}

      return stateEstabilizaAcelera;  
}


StartStop::StatesStartStop StartStop::ligaMotorSS (Motor &motor, Display &display) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    Motor::statesEngine status = motor.ligaMotor(display);

    // VOLTA PARA ELA MESMA ENQUANTO PROCESSA
    if (status == Motor::accelerating) { return stateLigaMotor; } 

    // PROCESSAMENTO FINALIZADO
    if (status == Motor::engineON ) {   
        tentativasLigar = 0;    
        return stateStart;
    } else { 
        timerTentativa = millis(); 
        return stateNotLigou; 
    }
}

StartStop::StatesStartStop StartStop::desligaMotorSS (Motor &motor, Display &display) {
    if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

    Motor::statesEngine status = motor.desligaMotor(display);

    // VOLTA PARA ELA MESMA ENQUANTO PROCESSA
    if (status == Motor::accelerating) { return stateDesligaMotor; } 

    // PROCESSAMENTO FINALIZADO
    if (status == Motor::engineOFF) { 
        tentativasDesligar = 0;      
        return stateStop;
    } else { 
        timerTentativa = millis(); // Correção: inicia o timer para o delay de notDesligou
        return stateNotDesligou; 
    }
}

StartStop::StatesStartStop StartStop::notLigou (Display &display) {
    if (tentativasLigar <= 2) {
        if (millis() - timerTentativa >= 1500) {
            tentativasLigar++;
            return stateLigaMotor;
        }
        return stateNotLigou;
    } else { 
        tentativasLigar = 0; // <-- ADICIONE ISTO AQUI
        return stateDesligaStartStop; }
}

StartStop::StatesStartStop StartStop::notDesligou (Display &display) {
    if (tentativasDesligar <= 2) {
        if (millis() - timerTentativa >= 1500) {
            tentativasDesligar++;
            return stateDesligaMotor;
        }
        return stateNotDesligou;
    } else { 
        tentativasDesligar = 0; // <-- ADICIONE ISTO AQUI
        return stateSwitchOFF; }
}

StartStop::StatesStartStop StartStop::freando () {
  if (digitalRead(switchSS) == HIGH) { return stateDesligaStartStop; }

  if (digitalRead(switchSS) == NOT_PRESSIONADO) { return stateDesligaStartStop; }

  return stateStop;
}
