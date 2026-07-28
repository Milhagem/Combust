#include "Display.hpp"

Hall hall;

void Display::iniciaDisplay () {
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando Ben 10");
  delay(500);
  this->lcd.clear();
  Wire.setClock(100000);  // Reduz a velocidade do I2C
}

void Display::mostraTensaoEVel(float velocidade, float tensao){
   if((millis() - timeOld) >= timeInterval){
    this->lcd.setCursor(0,0);
    this->lcd.print("Acc:   ");
    this->lcd.setCursor(5,0);
    this->lcd.print(hall.getAceleracao()); // Assumindo que este método exista

    this->lcd.setCursor(0,1);
    this->lcd.print("Vel:   ");
    this->lcd.setCursor(6,1);
    this->lcd.print(velocidade);

    this->lcd.setCursor(0,1);
    this->lcd.setCursor(12,1);
    this->lcd.print(tensao); 

    timeOld = millis();
  }
}

void Display::atualizaDisplay(float velocidade, int FSMState, float tensao) {

  String FSMState_str;
  
  if ((millis() - timeOld) >= timeInterval) {

  mostraTensaoEVel(velocidade, tensao);

  switch (FSMState)
  {
    case StartStop::stateSwitchOFF:
      FSMState_str = "SS_off";
      break;

    case StartStop::stateSwitchON:
      FSMState_str = "SS__on";
      break;
      
    case StartStop::stateDesligaMotor:
      FSMState_str = "deslgM";
      break;
      
    case StartStop::stateLigaMotor:
      FSMState_str = "ligaM ";
      break;
      
    case StartStop::stateEstabilizaAcelera:
      FSMState_str = "estabA";
      break;

    // case StartStop::stateManipulaBorboleta:
    //   FSMState_str = "manipB";
    //   break;

    case StartStop::stateStart:
      FSMState_str = "iniciou";
      break;

    case StartStop::stateStop:
      FSMState_str = "parou";
      break;
      
    case StartStop::stateFreando:
      FSMState_str = "freou";
      break;

    case StartStop::stateNotLigou:
      FSMState_str = "notLig";
      break;

    case StartStop::stateNotDesligou:
      FSMState_str = "notDsg";
      break;
      
    case StartStop::stateDesligaStartStop:
      FSMState_str = "deslgSS";
      break;
      
    default:
      break;
  }
  
  this->lcd.setCursor(10, 0);
  this->lcd.print(FSMState_str);
  
  }
}
