#include "Motor.hpp"
#include "Display.hpp"
#include "Velocidade.hpp"


void Motor::Parametros_setup_controle_motor(){
  pinMode(PIN_LIGA_MOTOR, OUTPUT);
  pinMode(PIN_DESLIGA_MOTOR, OUTPUT);
  digitalWrite(PIN_LIGA_MOTOR, LOW);
  digitalWrite(PIN_DESLIGA_MOTOR, LOW);
}

Sensores_motor::statesEngine Motor::ligaMotor(Display& display, Sensores_motor& sensores){
  
  if (sensores.analisa_status_motor() == Sensores_motor::engineOFF) {
    const unsigned long tempoMaxPartida = 4000; // ms
    unsigned long timerPartida = millis();      // ms

    BSFC::Escreve_servo(POS_SERVO_PARTIDA);
    posServoAtual = POS_SERVO_PARTIDA;
    
    digitalWrite(PIN_LIGA_MOTOR, HIGH);

    while(millis() - timerPartida <= tempoMaxPartida) {
      display.mostraTensaoEVel(Velocidade::calculaVelocidade(), sensores); 
      
      float tensao = sensores.analisaTensao(); 
      if (tensao > tensaoMotorON) {
        digitalWrite(PIN_LIGA_MOTOR, LOW);
        return Sensores_motor::engineON;
      }
    }

    if (sensores.analisa_status_motor() == Sensores_motor::engineOFF) {
      digitalWrite(PIN_LIGA_MOTOR, LOW);
      return Sensores_motor::engineOFF;
    } else {
      digitalWrite(PIN_LIGA_MOTOR, LOW); 
      return Sensores_motor::engineON;
    }
  }
  return Sensores_motor::engineON; 
}

Sensores_motor::statesEngine Motor::desligaMotor(Display& display, Sensores_motor& sensores){
  
  BSFC::Escreve_servo(POS_SERVO_FECHADA);
  posServoAtual = POS_SERVO_FECHADA;
  
  if (sensores.analisa_status_motor() == Sensores_motor::engineON) {

    digitalWrite(PIN_DESLIGA_MOTOR, HIGH);    // Relé aberto

    const unsigned long tempoInjecaoAberta = 4000;     // ms
    unsigned long timerInjecaoAberta = millis();       // ms

    while(millis() - timerInjecaoAberta <= tempoInjecaoAberta) {
      display.mostraTensaoEVel(Velocidade::calculaVelocidade(), sensores);
      
      if (sensores.analisa_status_motor() == Sensores_motor::engineOFF && millis() - timerInjecaoAberta >= 1000) {
        digitalWrite(PIN_DESLIGA_MOTOR, LOW);
        return Sensores_motor::engineOFF;
      }      
    }

    if (sensores.analisa_status_motor() == Sensores_motor::engineOFF ) {
      digitalWrite(PIN_DESLIGA_MOTOR, LOW);
      return Sensores_motor::engineOFF;
    } else {
      digitalWrite(PIN_DESLIGA_MOTOR, LOW); 
      return Sensores_motor::engineON;
    }    
  }
  return Sensores_motor::engineOFF;
}