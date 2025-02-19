#ifndef STARTSTOP_H
#define STARTSTOP_H

#include "Motor.hpp" 
#include "Velocidade.hpp"

#define erroAcel 0.1
#define aceleraçãoIdeal 0.5

class StartStop {
public:
    enum StatesStartStop {
        stateSwitchON,
        stateSwitchOFF,
        stateLigaMotor,
        stateDesligaMotor,
        stateEstabilizaAceleração,
        stateEstabilizaVel,
        stateManipulaBorboleta,
        stateStart,
        stateStop,
        stateFreando,
        stateNãoLigou
    };

    StartStop  ();

    static float getVelocidadeMaxVariavel ();

    static void setVelocidadeMaxVariavel (float &velMax);

    static StatesStartStop switchOFF ();

    static StatesStartStop switchON (); 

    static StatesStartStop ligaMotor ();

    static StatesStartStop desligaMotor ();

    static StatesStartStop estabilizaAceleração ();
    
    static StatesStartStop estabilizaVel ();
    
    static StatesStartStop manipulaBorboleta ();

    static StatesStartStop start ();
    
    static StatesStartStop stop ();

    static StatesStartStop freando ();

    static StatesStartStop nãoLigou ();

};

#endif