#ifndef STARTSTOP_H
#define STARTSTOP_H

#include "Motor.hpp" 
#include "Velocidade.hpp"

#define erroDeAceitaçao 0.1
#define aceleraçãoIdeal 0.5
#define posServoInicial 15
#define velocidadeMinima 5
#define velocidadeMax 30

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

    enum StatesManipulaBorboleta {
        manterAceleração,
        manterVelocidadeAcima,
        manterVelocidadeAbaixo
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

    static StatesStartStop start (Motor &motor);
    
    static StatesStartStop stop (Motor &motor);

    static StatesStartStop freando ();

    static StatesStartStop nãoLigou ();

private:
    static StatesManipulaBorboleta borboleta;

};


#endif