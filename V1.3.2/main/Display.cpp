#include "Display.hpp"

volatile unsigned long timeOld = 0;

//Display::Display () {
//  this->lcd.init();
//  this->lcd.backlight();
//  this->lcd.setCursor(0,0);
//  this->lcd.print("Iniciando M10");
//  delay(500);
//  
//}

void Display:: iniciaDisplay () {
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando M10");
  delay(500);
  this->lcd.clear();
  Wire.setClock(100000);  // Reduz a velocidade do I2C
}


void Display::mostraTensaoEVel(float velocidade){
  if ( (millis() - timeOld) >= timeInterval){
    //this->lcd.setCursor(0,0);
    //this->lcd.print("DDP:    ");
    //this->lcd.setCursor(5,0);
    //this->lcd.print(motor.analisaTensao());
    // Exibir velocidade
    this->lcd.setCursor(0,1);
    this->lcd.print("Vel:            ");
    this->lcd.setCursor(6,1);
    this->lcd.print(velocidade);

    timeOld = millis();
  }
}

void Display::atualizaDisplay( float velocidade, int FSMState) {
  mostraTensaoEVel(velocidade);

  String FSMState_str;
if ( (millis() - timeOld) >= timeInterval){

  switch (FSMState)
  {
    case 1:
      FSMState_str = "SS_off";
      break;

    case 2:
      FSMState_str = "SS__on";
      break;
      
    case 3:
      FSMState_str = "deslgM";
      break;
      
    case 4:
      FSMState_str = "ligaM ";
      break;
      
    case 5:
      FSMState_str = "estabA";
      break;
      
    case 6:
      FSMState_str = "estabV";
      break;

    case 7:
      FSMState_str = "manipB";
      break;

    case 8:
      FSMState_str = "iniciou";
      break;

    case 9:
      FSMState_str = "parou";
      break;
      
    case 10:
      FSMState_str = "freou";
      break;

    case 11:
      FSMState_str = "notLig";
      break;

    case 12:
      FSMState_str = "notDsg";
      break;
      
    case 13:
      FSMState_str = "deslgSS";
      break;
      
    default:
      break;
  }

  this->lcd.setCursor(10, 0);
  this->lcd.print(FSMState_str);

  this->lcd.setCursor(15,1);
  //this->lcd.print(motor.checaEstadoMotor());
}
}
