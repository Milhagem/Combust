#include "Motor.h"

#include "Wire.h"
#include "LiquidCrystal_I2C.h"

/**
 * Conferir  a funcao analisaTesao();  Aqueles calculos estao corretos?
*/

/**
 * Pode ser que o codigo "trave" no ligaMotor()
*/


#define pedalGND  A8
#define pedalVcc A10
#define pinServo   8
#define freio    A15
#define switchSS A14

#define FALSE         0
#define TRUE          1
#define PRESSIONADO   0
#define LEDVERMELHO   2
#define time_interval 200 //ms

#define stateSS_off       0
#define stateMonitoraVec  1
#define stateIncrementVec 2
#define stateDesligaMotor 3
#define stateLigaMotor    4
#define stateFreiando     5

Motor motor;
LiquidCrystal_I2C lcd(0x27,16,2);

int FSMstate = stateSS_off;
int valorInicial; 
float tensao;
int pos;
boolean estadoMotor;
unsigned int timeold;

void setup() {
  motor.servoAttach(pinServo);
  
  pinMode(pinLigaMotor,OUTPUT);
  pinMode(pinDesligaMotor,OUTPUT);
  pinMode(freio,INPUT_PULLUP);
  pinMode(switchSS,INPUT_PULLUP);
  pinMode(vecAtual,INPUT); 
  pinMode(LM2907,INPUT);

  digitalWrite(pinLigaMotor,LOW);
  digitalWrite(pinDesligaMotor,LOW);
  analogWrite(pedalGND, 0); // Pino 8 -> GND Pedal
  analogWrite(pedalVcc, 1023); // Pino 10 -> Vcc Pedal

  motor.servoWrite(0);  // Poe o servo na posicao inicial
  estadoMotor = DESLIGADO;

  lcd.init();                   // Inicializando o LCD
  lcd.backlight();              // Inicializando o backlight do LCD
  lcd.setCursor(0,0);
  lcd.print("Iniciando CR7");    // Exibir na tela
  delay(500);

  Serial.begin(9600);
}


void loop() {
  switch (FSMstate)
  { 
    case stateSS_off:
      Serial.println("FSMstate = StartStop OFF");
      Serial.print("Velocidade:" );
      Serial.println(analogRead(vecAtual));
  

      if(digitalRead(switchSS) == LOW && analogRead(vecAtual)>vecMin){
        FSMstate = stateMonitoraVec;
      }
    break;
  

    case stateMonitoraVec: 
      Serial.println("FSMstate = Monitora velocidade");

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO) {
          FSMstate = stateFreiando;
        }

        if(analogRead(vecAtual)<vecMax && analogRead(vecAtual)>ZEROvec &&
           digitalRead(freio) != PRESSIONADO) {
            if(estadoMotor==DESLIGADO)
              FSMstate = stateLigaMotor;
            else
              FSMstate = stateIncrementVec;
        } else if(analogRead(vecAtual)>vecMax && digitalRead(freio) != PRESSIONADO) {
          FSMstate = stateDesligaMotor;
        }
      } else { FSMstate = motor.desligaStartStop(); }

      
    break;
  

    case stateIncrementVec: 
      Serial.println("FSMstate = Incrementa Velocidade");

      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) != PRESSIONADO){
          if(pos <= 80) {
            pos+=10;
            motor.servoWrite(pos);
            motor.printVelocidade();
            Serial.print("    ; Posicao: "); 
            Serial.println(pos);
            Serial.println("Motor ligado");

            FSMstate = stateMonitoraVec;
          }
        } else{
          FSMstate = stateFreiando;
        }
      } 
      else { FSMstate = motor.desligaStartStop(); }
       
    break;
  

    case stateDesligaMotor:
      Serial.println("FSMstate = Desliga Motor");
      motor.desligaMotor();
      pos+=0;
      motor.servoWrite(pos);
      FSMstate = stateMonitoraVec; 

    break;
  

    case stateLigaMotor:
      Serial.println("FSMstate = Liga Motor"); 

      if(digitalRead(switchSS) == LOW){
        if(digitalRead(freio) != PRESSIONADO) {
          motor.ligaMotor();
          FSMstate = stateMonitoraVec;
        } else if(digitalRead(freio) == PRESSIONADO){
          FSMstate = stateFreiando;
        }    
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  

    case stateFreiando:
      Serial.println("FSMstate = Freio Pressionado"); 
      if(digitalRead(switchSS) == LOW) {
        if(digitalRead(freio) == PRESSIONADO){
          digitalWrite(LEDVERMELHO,HIGH);
          pos = 0;
          motor.servoWrite(pos);
        } else if(digitalRead(freio) != PRESSIONADO ){
          digitalWrite(LEDVERMELHO,LOW);
          FSMstate = stateMonitoraVec;
        }
      } else { FSMstate = motor.desligaStartStop(); }

    break;
  
    default: FSMstate = stateSS_off;

  }

  if (millis() - timeold >=time_interval){ //Atualiza o display com Tensão e Velocidade. Taxa de atualização = Time interval
    
    lcd.setCursor(0,0);
    lcd.print("Tensao:         ");    // Exibir leitura LM
    lcd.setCursor(8,0);
    lcd.print(motor.analisaTensao()); 
    lcd.setCursor(0,1);
    lcd.print("Velocidade:     ");    // Exibir leitura velocidade
    lcd.setCursor(12,1);
    lcd.print(analogRead(vecAtual));
    
    timeold = millis();
  }

  estadoMotor=motor.checaEstadoMotor();// Atualiza o estado do motor
}
