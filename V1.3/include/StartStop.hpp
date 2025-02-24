#ifndef STARTSTOP_H
#define STARTSTOP_H

#include "Motor.hpp" 
#include "Velocidade.hpp"
#include "Display.hpp"

#define erroDeAceitaçao 0.1
#define aceleraçãoIdeal 0.5
#define posServoInicial 15
#define velocidadeMinima 5
#define velocidadeMax 30
#define tempoImcrementoIdeal 100
#define velZERO 0
#define PRESSIONADO 1
#define NOT_PRESSIONADO 0   
#define switchSS 20
#define pinFreio 21
#define tempoMaximoVelocidade 10000

class StartStop {
public:
    enum StatesStartStop {
        stateSwitchON,
        stateSwitchOFF,
        stateLigaMotor,
        stateDesligaMotor,
        stateEstabilizaAceleração,
        stateEstabilizaVelocidade,
        stateManipulaBorboleta,
        stateStart,
        stateStop,
        stateFreando,
        stateNãoLigou,
        stateNãoDesligou,
        stateDesligaStartStop
    };

    enum StatesManipulaBorboleta {
        manterAceleração,
        manterVelocidadeAcima,
        manterVelocidadeAbaixo
    };

    static StatesStartStop switchOFF ();

    static StatesStartStop switchON (); 

    static StatesStartStop ligaMotor (Motor &motor, Display &display);

    static StatesStartStop desligaMotor (Motor &motor, Display &display);

    static StatesStartStop estabilizaAceleração (Motor &motor);
    
    static StatesStartStop estabilizaVelocidade (Motor &motor);
    
    static StatesStartStop manipulaBorboleta ();

    static StatesStartStop start (Motor &motor);
    
    static StatesStartStop stop (Motor &motor);

    static StatesStartStop freando ();

    static StatesStartStop nãoLigou (Display &display);
    
    static StatesStartStop nãoDesligou (Display &display);

    static StatesStartStop desligaStartStop (Display &display);

private:
    static StatesManipulaBorboleta borboleta;
    static int tentativasLigar;
    static int tentativasDesligar;
    static bool inicioVel;
    static float tempoInicioVel;

};


#endif