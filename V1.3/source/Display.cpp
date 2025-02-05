#include "Display.h"

void Display::iniciaDisplay(){
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando M10");
  delay(500);
  
}

void Display::mostraTensaoEVel(Motor &motor, int velocidade){
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

void Display::atualizaDisplay(Motor &motor, int velocidade, StartStop::StatesStartStop FSMState){
  mostraTensaoEVel(motor, velocidade);

  std::string FSMState_str;

  switch (FSMState)
  {
    case StartStop::stateSwitchOFF:
      FSMState = "SS_off";
      break;

    case StartStop::stateSwitchON:
      FSMState = "SS__on";
      break;
    
    case StartStop::stateMonitoraVel:
      FSMState = "monitV";
      break;

    case StartStop::stateIncrementaVel:
      FSMState = "incrmV";
      break;

    case StartStop::stateDesligaMotor:
      FSMState = "deslgM";
      break;

    case StartStop::stateLigaMotor:
      FSMState = "ligaM ";
      break;

    case StartStop::stateFreando:
      FSMState = "freou";
      break;

    default:
      break;
  }

  lcd.setCursor(10, 0);
  lcd.print(FSMState);

  lcd.setCursor(15,1);
  lcd.print(motor.getEstadoMotor());
  }
