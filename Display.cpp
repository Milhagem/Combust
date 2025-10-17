#include "Display.hpp"
#include "Velocidade.hpp"
volatile unsigned long timeOld = 0;

//Display::Display () {
//  this->lcd.init();
//  this->lcd.backlight();
//  this->lcd.setCursor(0,0);
//  this->lcd.print("Iniciando M10");
//  delay(500);
//  
//}

void Display::iniciaDisplay () {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Iniciando M10");
  delay(500);
  lcd.clear();
  Wire.setClock(100000);  // Reduz a velocidade do I2C
}


void Display::mostraTensaoEVel(float velocidade, float acelera){
   //if ( (millis() - timeOld) >= timeInterval){
    this->lcd.setCursor(0,0);
    this->lcd.print("ace:         ");
    this->lcd.setCursor(5,0);
    this->lcd.print(acelera);

    this->lcd.setCursor(0,1);
    this->lcd.print("Vel:            ");
    this->lcd.setCursor(6,1);
    this->lcd.print(velocidade);

    this->lcd.setCursor(12,1);
    this->lcd.print(Velocidade::velocidade_rpm);
    timeOld = millis();

  //}
}

void Display::atualizaDisplay(float velocidade, float acelera) {

if ( (millis() - timeOld) >= timeInterval){

  mostraTensaoEVel(velocidade, acelera);
  //this->lcd.setCursor(15,1);
  //this->lcd.print(motor.checaEstadoMotor());
}
}
