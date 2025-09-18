#include "Motor.hpp" 
#include "Velocidade.hpp"
#include "Display.hpp"
#include "StartStop.hpp"
#include <PinChangeInterrupt.h> // Incluindo a nova biblioteca

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
float tempoIncrementoInicial = 0;
float point_velocidade = 0;
float point_aceleracao = 0;

//extern int __heap_start, *__brkval;
//int freeMemory() {
//    int v;
//    return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
//}

Motor motor;
Display display; 
unsigned int time = 0;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);

StartStop::StatesStartStop FSMstate = StartStop::stateSwitchOFF;

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
//-----------------------------------------Variaveis de teste
    timerCalcVel = 0;
    lastTimerCalcVel = 0;
    pulseInterval = 0;
    lastPulseInterval = 0;
    for (int i = 0; i < sampleSize; i++) { pulseIntervals[i] = 200; }
    pulseIndex = 0;
  
    attachInterrupt (digitalPinToInterrupt (pinSensorHall), Velocidade::calc, RISING);
    //attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(10), Velocidade::calc, RISING); 
  
    //------------------------------------------------------------------
  
    
    // Display LCD
    display.iniciaDisplay();
    //lcd.backlight();

    motor.servoAttach(pinServo);
    //servo.writeMicroseconds(1000);
    motor.servoWrite(posServoInicial);

//    Serial.print("Memória livre (bytes): ");
//    Serial.println(freeMemory());

  }

void loop() {
    display.atualizaDisplay (Velocidade::calculaVelocidade(point_aceleracao),FSMstate, point_aceleracao, motor);
    switch (FSMstate) {
        case StartStop::stateSwitchOFF:
            FSMstate = StartStop::switchOFF();
            break;
        case StartStop::stateSwitchON:
            FSMstate = StartStop::switchON();
            break;
        case StartStop::stateLigaMotor:
            FSMstate = StartStop::ligaMotorSS(motor, display);
            break;
        case StartStop::stateDesligaMotor:
            FSMstate = StartStop::desligaMotorSS(motor, display);
            break;
        case StartStop::stateEstabilizaAcelera:
            FSMstate = StartStop::estabilizaAcelera(motor);
            break;
        case StartStop::stateManipulaBorboleta:
            FSMstate = StartStop::manipulaBorboleta(motor, tempoUltimoIncremento);
            break;
        case StartStop::stateStart:
            FSMstate = StartStop::start(motor, tempoIncrementoInicial);
            break;
        case StartStop::stateStop:
            FSMstate = StartStop::stop(motor);
            break;
        case StartStop::stateFreando:
            FSMstate = StartStop::freando();
            break;
        case StartStop::stateDesligaStartStop:
            FSMstate = StartStop::desligaStartStop(motor, display);
            break;
        case StartStop::stateNotLigou:
            FSMstate = StartStop::notLigou(display);
            break;
        case StartStop::stateNotDesligou:    
            FSMstate = StartStop::notDesligou(display);
            break;    
        default:
            FSMstate = StartStop::stateDesligaStartStop;
            break;
    }  
}  
   