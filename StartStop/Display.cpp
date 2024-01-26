#include "Display.h"


#define stateSS_off       0
#define stateSS_on        1
#define stateMonitoraVel  2
#define stateIncrementVel 3
#define stateDesligaMotor 4
#define stateLigaMotor    5
#define stateFreiando     6

void Display::iniciaDisplay(){
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando CR7");
  delay(500);
}

void Display::mostraTensaoEVel(Motor &motor, int generic_number2){
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
    lcd.print(analogRead(velAtual));

    timeOld = millis();
  }
}

void Display::atualizaDisplay(Motor &motor, int FSMState_int, int generic_number){
  mostraTensaoEVel(motor, generic_number);

  String FSMState;

  switch (FSMState_int)
  {
    case stateSS_off:
      FSMState = "SS_off";
      break;

    case stateSS_on:
      FSMState = "SS__on";
      break;
    
    case stateMonitoraVel:
      FSMState = "monitV";
      break;

    case stateIncrementVel:
      FSMState = "incrmV";
      break;

    case stateDesligaMotor:
      FSMState = "deslgM";
      break;

    case stateLigaMotor:
      FSMState = "ligaM";
      break;

    case stateFreiando:
      FSMState = "freou";
      break;

    default: 0;
      break;
  }

  lcd.setCursor(10, 0);
  lcd.print(FSMState);

  lcd.setCursor(15,1);
  lcd.print(motor.getEstadoMotor());
  }
