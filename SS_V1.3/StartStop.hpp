#ifndef STARTSTOP_H
#define STARTSTOP_H

#include "Motor.hpp" 
#include "Velocidade.hpp"
#include "Display.hpp"
//#include <cmath>

class Display;
class Motor;


#define erroAceitavel 0.1
#define aceleraIdeal 0.5
#define velocidadeMinima 5
#define velocidadeMax 15
#define velZERO 0
#define PRESSIONADO 1
#define NOT_PRESSIONADO 0   
#define switchSS 4
#define pinFreio 21
#define tempoMaximoVelocidade 10000
#define tempoIncrementoIdealMax 600
#define tempoIncrementoIdealMin 300


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

    static float getA () {return a; }

    static StatesStartStop ligaMotorSS (Motor &motor, Display &display);

    static StatesStartStop desligaMotorSS (Motor &motor, Display &display);

    static StatesStartStop estabilizaAcelera (Motor &motor);
    
//  static StatesStartStop estabilizaVelocidade (Motor &motor);
    
    static StatesStartStop manipulaBorboleta (Motor &motor, float &tempoUltimoImcremento);

    static StatesStartStop start (Motor &motor);
    
    static StatesStartStop stop (Motor &motor);

    static StatesStartStop freando ();

    static StatesStartStop notLigou (Display &display);
    
    static StatesStartStop notDesligou (Display &display);

    static StatesStartStop desligaStartStop (Motor &motor, Display &display);

    static float tempBorbIdeal();

private:
//    static StatesManipulaBorboleta borboleta;
    static int tentativasLigar;
    static int tentativasDesligar;
    static bool inicioVel;
    static float tempoInicioVel;
// a é a variavel que faz parte da função velBorb
    static float a;

    static int testeBorb;

};


#endif