#include "Motor.hpp"

Hall hall;

void Motor::Parametros_setup_controle_e_sensores_motor(){
  pinMode(PIN_LIGA_MOTOR, OUTPUT);
  pinMode(PIN_DESLIGA_MOTOR, OUTPUT);
  digitalWrite(PIN_LIGA_MOTOR, LOW);
  digitalWrite(PIN_DESLIGA_MOTOR, LOW);
  CKP::Inicializar_setup_sensores_motor();
}

Motor::statesEngine Motor::ligaMotor(Display& display){
  // Removido o 'static' daqui para usar as variáveis do seu Motor.hpp
  if (!acionando) {
    acionando = true;
    timerPartida = millis();
    Servo::Escreve_servo(POS_SERVO_PARTIDA);
    posServoAtual = POS_SERVO_PARTIDA;
    digitalWrite(PIN_LIGA_MOTOR, HIGH);
  }

  display.mostraTensaoEVel(hall.getVelocidade(), LM2907::getTensao()); 
  
  if (LM2907::analisaTensao() > tensaoMotorON) {
    acionando = false;
    digitalWrite(PIN_LIGA_MOTOR, LOW);
    return Motor::engineON; 
  }

  if (millis() - timerPartida > 4000) {
    acionando = false;
    digitalWrite(PIN_LIGA_MOTOR, LOW);
    return Motor::engineOFF; 
  }

  return Motor::accelerating; 
}

Motor::statesEngine Motor::desligaMotor(Display& display){
  static unsigned long timerInjecao = 0;
  static bool desligando = false;

  // TRAVA DE SEGURANÇA: Cancela a partida caso a FSM tenha abortado o ligaMotor no meio do processo
  acionando = false;
  digitalWrite(PIN_LIGA_MOTOR, LOW);

  if (!desligando) {
    desligando = true;
    timerInjecao = millis();
    Servo::Escreve_servo(POS_SERVO_FECHADA);
    posServoAtual = POS_SERVO_FECHADA;
    digitalWrite(PIN_DESLIGA_MOTOR, HIGH); 
  }

  display.mostraTensaoEVel(hall.getVelocidade(), LM2907::getTensao());

  if (Motor::analisa_status_motor() == Motor::engineOFF && millis() - timerInjecao >= 1000) {
    desligando = false;
    digitalWrite(PIN_DESLIGA_MOTOR, LOW);
    return Motor::engineOFF;
  }

  if (millis() - timerInjecao > 4000) {
    desligando = false;
    digitalWrite(PIN_DESLIGA_MOTOR, LOW);
    return Motor::engineON; 
  }

  return Motor::accelerating; 
}

void Motor::analisa_status_central() {
    if (MAP::return_status_map() || TPS::return_status_tps()) {
        status_central = true;
        tempo_central_desligada = 0; 
    } else {
        if(tempo_central_desligada == 0) {
            tempo_central_desligada = millis();
        } else if (millis() - tempo_central_desligada > 2000) {
            status_central = false; 
        }
    }
}

Motor::statesEngine Motor::analisa_status_motor() {
    if (LM2907::getTensao() > tensaoMotorON) {
        return engineON;
    } else {
        return engineOFF;
    }
}

void Motor::analisa_sensores_motor(){
    Motor::analisa_status_central();
    Motor::analisa_status_motor();
    Lambda::analisaLambda();
    MAP::analisaMap();
    TPS::analisaPosBorbo();
    LM2907::analisaTensao();
}