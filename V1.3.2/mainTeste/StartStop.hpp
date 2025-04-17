#ifndef STARTSTOP_H
#define STARTSTOP_H

#include "Motor.hpp" 
#include "Velocidade.hpp"
#include "Display.hpp"

class Display;
class Motor;


#define erroAceitavel 0.1
#define aceleraIdeal 0.5
#define posServoInicial 0
#define velocidadeMinima 0
#define velocidadeMax 10
#define tempoIncrementoIdeal 200
#define velZERO 0
#define PRESSIONADO 1
#define NOT_PRESSIONADO 0   
#define switchSS 4
#define pinFreio 21
#define tempoMaximoVelocidade 10000

class StartStop {
public:
    enum StatesStartStop {
        stateSwitchON,
        stateSwitchOFF,
        stateLigaMotor,
        stateDesligaMotor,
        stateEstabilizaAcelera,
        stateEstabilizaVelocidade,
        stateManipulaBorboleta,
        stateStart,
        stateStop,
        stateFreando,
        stateNotLigou,
        stateNotDesligou,
        stateDesligaStartStop
    };

//    enum StatesManipulaBorboleta {
//        manterAcelera,
//        manterVelocidadeAcima,
//        manterVelocidadeAbaixo
//    };

    static StatesStartStop switchOFF ();

    static StatesStartStop switchON (); 

    static StatesStartStop ligaMotorSS (Motor &motor, Display &display);

    static StatesStartStop desligaMotorSS (Motor &motor, Display &display);

    static StatesStartStop estabilizaAcelera (Motor &motor);
    
//    static StatesStartStop estabilizaVelocidade (Motor &motor);
    
    static StatesStartStop manipulaBorboleta (Motor &motor, float &tempoUltimoImcremento);

    static StatesStartStop start (Motor &motor);
    
    static StatesStartStop stop (Motor &motor);

    static StatesStartStop freando ();

    static StatesStartStop notLigou (Display &display);
    
    static StatesStartStop notDesligou (Display &display);

    static StatesStartStop desligaStartStop (Motor &motor, Display &display);

private:
//    static StatesManipulaBorboleta borboleta;
    static int tentativasLigar;
    static int tentativasDesligar;
    static bool inicioVel;
    static float tempoInicioVel;

};


#endif