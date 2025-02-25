#include "Display.h"

void Display::iniciaDisplay(){
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando M10");
  delay(500);
  
}

void Display::mostraTensaoEVel(Motor &motor, float velocidade){
    if ( (millis()-timeOld) >= timeInterval){

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

void Display::atualizaDisplay(Motor &motor, float velocidade, StartStop::StatesStartStop FSMState){
  mostraTensaoEVel(motor, velocidade);

  std::string FSMState_str;

  switch (FSMState)
  {
    case StartStop::stateSwitchOFF:
      FSMState_str = "SS_off";
      break;

    case StartStop::stateSwitchON:
      FSMState_str = "SS__on";
      break;
    
    case StartStop::stateMonitoraVel:
      FSMState_str = "monitV";
      break;

    case StartStop::stateIncrementaVel:
      FSMState_str = "incrmV";
      break;

    case StartStop::stateDesligaMotor:
      FSMState_str = "deslgM";
      break;

    case StartStop::stateLigaMotor:
      FSMState_str = "ligaM ";
      break;

    case StartStop::stateFreando:
      FSMState_str = "freou";
      break;

    default:
      break;
  }

  lcd.setCursor(10, 0);
  lcd.print(FSMState_str);

  lcd.setCursor(15,1);
  lcd.print(motor.checaEstadoMotor());
  }
