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

#define desligaSS     x0
#define stateSS_on    1
#define LigaMotor     2
#define Aguarda       3
#define ManipulaServo 4
#define MantemVelMax  5

void Display:: iniciaDisplay () {
  this->lcd.init();
  this->lcd.backlight();
  this->lcd.setCursor(0,0);
  this->lcd.print("Iniciando M10");
  delay(500);
}

void Display::mostraTensaoEVel(Motor &motor, int velocidade) {
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

void Display::atualizaDisplay(Motor &motor, int velocidade, int FSMState_int) {
  mostraTensaoEVel(motor, velocidade);

  String FSMState;

  switch (FSMState_int)
  {
    case desligaSS:
      FSMState = "desligaSS";
      break;

    case stateSS_on:
      FSMState = "SS__on";
      break;

    case LigaMotor:
      FSMState = "LigaMotor";
      break;

    case Aguarda:
      FSMState = "Aguarda";
      break;

    case ManipulaServo:
      FSMState = "ManipulaServo";
      break;

    case MantemVelMax:
      FSMState = "MantemVelMax";
      break;

    default: 0;
      break;
  }

  lcd.setCursor(10, 0);
  lcd.print(FSMState);

  lcd.setCursor(15,1);
  lcd.print(motor.getEstadoMotor());
  }
