#include "Motor.hpp" 
#include "Velocidade.hpp"
#include "Display.hpp"
#include "StartStop.hpp"

extern unsigned long timerIncrementoServo;
extern volatile unsigned long pulseInterval;     // ms
extern volatile unsigned long lastPulseInterval; // ms
extern volatile unsigned long timerCalcVel;      // ms
extern volatile unsigned long lastTimerCalcVel;  // ms
extern volatile unsigned long timeEstabilizaVel;
extern volatile unsigned long timeEstabilizacao; 
extern volatile unsigned long pulseIntervals[sampleSize];
extern volatile int pulseIndex;
extern unsigned long lastTimerTax;

float tempoUltimoIncremento = 0;

StartStop::StatesStartStop FSMstate = StartStop::stateManipulaBorboleta;

//extern int __heap_start, *__brkval;
//int freeMemory() {
//    int v;
//    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
//}

Motor motor;
Display display; 
unsigned int time = 0; 

void setup() {
   Serial.begin(9600);
    
    pinMode(pinLigaMotor, OUTPUT);
    pinMode(pinDesligaMotor, OUTPUT);
    //pinMode(pinVelPedal, INPUT); 
    pinMode(LM2907, INPUT);
    pinMode(pinSensorHall, INPUT);
    pinMode(pinFreio, INPUT_PULLUP);
    pinMode(switchSS, INPUT_PULLUP);
  
    
    digitalWrite(pinLigaMotor, LOW);
    digitalWrite(pinDesligaMotor, LOW);
    // analogWrite(pinPedalGND, 0);    // Pino A8  -> GND Pedal
    // analogWrite(pinPedalVCC, 1023); // Pino A10 -> VCC Pedal
    
    //-----------------------------------------------Variaveis de teste
    timerCalcVel = 0;
    lastTimerCalcVel = 0;
    pulseInterval = 0;
    lastPulseInterval = 0;
    for (int i = 0; i < sampleSize; i++) { pulseIntervals[i] = 0; }
    pulseIndex = 0;
  
    attachInterrupt (digitalPinToInterrupt (pinSensorHall), Velocidade::calc, RISING);
  
    //------------------------------------------------------------------
  
    
    // Display LCD
    display.iniciaDisplay();

//    Serial.print("Memória livre (bytes): ");
//    Serial.println(freeMemory());

  }

void loop() {
    display.atualizaDisplay (Velocidade::calculaVelocidade(), FSMstate);
    // delay(3000);
    switch (FSMstate) {
        case StartStop::stateSwitchOFF:
          FSMstate = StartStop::switchOFF();
          break;
        case StartStop::stateSwitchON:
              delay(3000);
            FSMstate = StartStop::switchON();
            break;
        case StartStop::stateLigaMotor:
             delay(3000);
            FSMstate = StartStop::ligaMotorSS(motor, display);
            break;
        case StartStop::stateDesligaMotor:
             delay(3000);

            FSMstate = StartStop::desligaMotorSS(motor, display);
            break;
        case StartStop::stateEstabilizaAcelera:
             delay(3000);

            FSMstate = StartStop::estabilizaAcelera(motor);
            break;
//        case StartStop::stateEstabilizaVelocidade:
//            FSMstate = StartStop::estabilizaVelocidade(motor);
//            break;
        case StartStop::stateManipulaBorboleta:
            //  delay(3000);

            FSMstate = StartStop::manipulaBorboleta(motor, tempoUltimoIncremento);
            break;
        case StartStop::stateStart:
             delay(3000);

            FSMstate = StartStop::start(motor);
            break;
        case StartStop::stateStop:
             delay(3000);

            FSMstate = StartStop::stop(motor);
            break;
        case StartStop::stateFreando:
               delay(3000);

            FSMstate = StartStop::freando();
            break;
        case StartStop::stateDesligaStartStop:
             delay(3000);

            FSMstate = StartStop::desligaStartStop(motor, display);
            break;
        case StartStop::stateNotLigou:
            FSMstate = StartStop::notLigou(display);
             delay(3000);

            break;
        case StartStop::stateNotDesligou:
             delay(3000);
    
            FSMstate = StartStop::notDesligou(display);
            break;    
        default:
            FSMstate = StartStop::stateDesligaStartStop;
    }  
}  
   