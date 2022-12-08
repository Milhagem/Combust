#include "Display.h"

void Display::iniciaDisplay(){
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando CR7");
  delay(500);
}

void Display::atualizaDisplay(Motor &motor){
    if ( (millis()-timeOld) >= timeInterval){
    
    // Exibir leitura LM2907
    lcd.setCursor(0,0);
    lcd.print("Tensao:         ");
    lcd.setCursor(8,0);
    lcd.print(motor.analisaTensao());

    // Exibir velocidade
    lcd.setCursor(0,1);
    lcd.print("Velocidade:     ");
    lcd.setCursor(12,1);
    lcd.print(analogRead(vecAtual));
    
    timeOld = millis();
  }

}