#ifndef STARTSTOP_H
#define STARTSTOP_H

#include "Motor.hpp" 
#include "Velocidade.hpp"

class StartStop {
public:
    enum StatesStartStop {
        stateSwitchON,
        stateSwitchOFF,
        stateLigaMotor,
        stateDesligaMotor,
        stateMonitoraVel,
        stateManipulaBorboleta,
        stateEstabilizaVel,
        stateFreando,
        stateNãoLigou
    };

    StartStop  ();

    static float getVelocidadeMaxVariavel ();

    static void setVelocidadeMaxVariavel (float &velMax);

    StatesStartStop switchOFF ();

    StatesStartStop switchON (); 

    StatesStartStop ligaMotor ();

    StatesStartStop desligaMotor ();

    StatesStartStop monitoraVel ();

    StatesStartStop manipulaBorboleta ();

    StatesStartStop estabilizaVel ();

    StatesStartStop freando ();

    StatesStartStop nãoLigou ();

private:
    static int velocidadeMaxVariavel;
    int tempoIncrementoBorboleta;
};

#endif