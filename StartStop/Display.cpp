#include "Display.h"

void Display::iniciaDisplay(){
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando CR7");
  delay(500);
}

void Display::mostraTensaoEVec(Motor &motor){
    if ( (millis()-timeOld) >= timeInterval){

    // Exibir leitura LM2907
    lcd.setCursor(0,0);
    lcd.print("DDP:    ");
    lcd.setCursor(5,0);
    lcd.print(motor.analisaTensao());

    // Exibir velocidade
    lcd.setCursor(0,1);
    lcd.print("Vec:    ");
    lcd.setCursor(6,1);
    lcd.print(analogRead(vecAtual));

    timeOld = millis();
  }
}

void Display::atualizaDisplay(Motor &motor){
  mostraTensaoEVec(motor);

  int FSMState_int = motor.getEstadoMotor();
  String FSMState;

  switch (FSMState_int)
  {
    case 0:
      FSMState = "SS_off";
      break;
    
    case 1:
      FSMState = "monitV";
      break;

    case 2:
      FSMState = "incrmV";
      break;

    case 3:
      FSMState = "deslgM";
      break;

    case 4:
      FSMState = "ligaM";

    case 5:
      FSMState = "freiou";

    default: 0;
      break;
  }

  lcd.setCursor(10, 0);
  lcd.print(FSMState);
  }
