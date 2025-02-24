#include "Motor.hpp" 
#include "Velocidade.hpp"
#include "Display.hpp"
#include "StartStop.hpp"

Motor motor;
Display display;    

void setup() {
    Serial.begin(115200);
    
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
  
    attachInterrupt (digitalPinToInterrupt (pinSensorHall), calc, RISING);
  
    //------------------------------------------------------------------
  
    
    // Display LCD
    display.iniciaDisplay();
  }

void loop() {
    StartStop::StatesStartStop FSMstate = StartStop::stateSwitchOFF;

    switch (FSMstate) {
        case StartStop::stateSwitchOFF:
            FSMstate = StartStop::switchOFF();
            break;
        case StartStop::stateSwitchON:
            FSMstate = StartStop::switchON();
            break;
        case StartStop::stateLigaMotor:
            FSMstate = StartStop::ligaMotor(motor, display);
            break;
        case StartStop::stateDesligaMotor:
            FSMstate = StartStop::desligaMotor(motor, display);
            break;
        case StartStop::stateEstabilizaAceleração:
            FSMstate = StartStop::estabilizaAceleração(motor);
            break;
        case StartStop::stateEstabilizaVelocidade:
            FSMstate = StartStop::estabilizaVelocidade(motor);
            break;
        case StartStop::stateManipulaBorboleta:
            FSMstate = StartStop::manipulaBorboleta();
            break;
        case StartStop::stateStart:
            FSMstate = StartStop::start(motor);
            break;
        case StartStop::stateStop:
            FSMstate = StartStop::stop(motor);
            break;
        case StartStop::stateFreando:
            FSMstate = StartStop::freando();
            break;
        case StartStop::stateDesligaStartStop:
            FSMstate = StartStop::desligaStartStop(display);
            break;
        default:
            FSMstate = StartStop::stateDesligaStartStop;
    }
}  
   