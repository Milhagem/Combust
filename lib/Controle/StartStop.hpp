#ifndef STARTSTOP_H
#define STARTSTOP_H

#include "Motor.hpp" 
#include "Display.hpp"
#include "Arduino.h"
#include "BSFC.hpp"
#include "Sensores_motor.hpp"
#include "Velocidade.hpp"


class Motor;
class Display;

#define erroAceitavel 0.1
#define velocidadeMinima 8      // Km/h
#define velocidadeMax 20        // Km/h
#define RPMideal 3500
#define PosBorboIdeal 4000

#define velZERO 0
#define PRESSIONADO 1
#define NOT_PRESSIONADO 0   

#define switchSS 16
#define pinFreio 7

#define tempoMaximoVelocidade 10000
#define tempoIncrementoIdealMax 600
#define tempoIncrementoIdealMin 300
#define tempoIncrementoIdeal 400



class StartStop {
public:

    enum StatesStartStop {
        stateSwitchON,
        stateSwitchOFF,
        stateLigaMotor,
        stateDesligaMotor,
        stateEstabilizaAcelera,
        stateEstabilizaVelocidade,
        stateStart,
        stateStop,
        stateFreando,
        stateNotLigou,
        stateNotDesligou,
        stateDesligaStartStop
    };


    static StatesStartStop switchOFF ();

    static StatesStartStop switchON (); 


    static StatesStartStop ligaMotorSS (Motor &motor, Display &display, Sensores_motor &sensores);
    static StatesStartStop desligaMotorSS (Motor &motor, Display &display, Sensores_motor &sensores);
    static StatesStartStop estabilizaAcelera (Motor &motor, Sensores_motor &sensores);
    static StatesStartStop estabilizaVelocidade (Motor &motor);
    static StatesStartStop start (Motor &motor, Sensores_motor &sensores);
    static StatesStartStop stop (Motor &motor, Sensores_motor &sensores);
    static StatesStartStop freando ();
    static StatesStartStop notLigou (Display &display);
    static StatesStartStop notDesligou (Display &display);
    static StatesStartStop desligaStartStop (Motor &motor, Display &display, Sensores_motor &sensores);

    // Procure por esta linha e mude para:
static void Inicializar_sensores_startstop();
    static int modoControle;

private:
    // static StatesManipulaBorboleta borboleta;
    static int tentativasLigar;
    static int tentativasDesligar;
    static bool inicioVel;
    static float tempoInicioVel;
    
   // Mudar isso aqui abaixo não sei pq
    static unsigned long timerTentativa;


    static int testeBorb;
};

#endif
