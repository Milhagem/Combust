#include "Display.h"

/*#define stateSS_off       0
#define stateSS_on        1
#define stateMonitoraVel  2
#define stateIncrementVel 3
#define stateDesligaMotor 4
#define stateLigaMotor    5
#define stateFreando      6
#define stateNaoLigou     7
#define estabilizaVel     8*/

/*#define desligaSS 0
#define stateSS_on        1
#define LigaMotor  2
#define Aguarda 3
#define ManipulaServo 4
#define MantemVelMax 5*/

#define stateSwitchON 0
#define stateSwitchOFF 1
#define stateLigaMotor 2
#define stateDesligaMotor 3
#define stateEstabilizaAcelera 4
#define stateEstabilizaVelocidade 5
#define stateManipulaBorboleta 6
#define stateStart 7
#define stateStop 8
#define stateFreando 9
#define stateNotLigou 10
#define stateNotDesligou 11
#define stateDesligaStartStop 12

void Display:: iniciaDisplay(){
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando M10");
  delay(500);
  
}

void Display::mostraTensaoEVel(Motor &motor, int velocidade){
    if ( (millis()-timeOld) >= timeInterval){

    // Exibir leitura LM2907
    lcd.setCursor(0,0);
    lcd.print("DDP:    ");
    lcd.setCursor(5,0);
    lcd.print(motor.analisaTensao());

    // Exibir velocidade
    lcd.setCursor(0,1);
    lcd.print("Vel:            ");
    lcd.setCursor(6,1);
    lcd.print(velocidade);

    timeOld = millis();
  }
}

void Display::atualizaDisplay(Motor &motor, int velocidade, int FSMState_int){
  mostraTensaoEVel(motor, velocidade);

  String FSMState;

  switch (FSMState_int)
  {
    case stateSwitchON:
      FSMState = "SSON";
      break;

    case stateSwitchOFF:
      FSMState = "SSOFF";
      break;

    case stateLigaMotor:
      FSMState = "LigaMotor";
      break;

    case stateDesligaMotor:
      FSMState = "DesMotor";
      break;

    case stateEstabilizaAcelera:
      FSMState = "EstabAcelera";
      break;

    case stateEstabilizaVelocidade:
      FSMState = "EstabVelocidade";
      break;

    case stateManipulaBorboleta:
      FSMState = "ManipBorboleta";
      break;
    
    case stateStart:
      FSMState = "stateStart";
      break;
    
    case stateStop:
      FSMState = "stateStop";
      break;
    
    case stateFreando:
      FSMState = "stateFreando";
      break;
    
    case stateNotLigou:
      FSMState = "stateNotLigou";
      break;
    
    case stateNotDesligou:
      FSMState = "NotDesligou";
      break;

    case stateDesligaStartStop:
      FSMState = "DesStartStop";
      break;

    default: 0;
      break;
  }

  lcd.setCursor(10, 0);
  lcd.print(FSMState);

  lcd.setCursor(15,1);
  lcd.print(motor.getEstadoMotor());

  }
