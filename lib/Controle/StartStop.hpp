#ifndef STARTSTOP_HPP
#define STARTSTOP_HPP

#include "Motor.hpp" 
#include "Display.hpp"
#include "Arduino.h"
#include "BSFC.hpp"
#include "Velocidade.hpp"
#include "Servo.hpp"

#define erroAceitavel 0.1

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

    //Variaveis manipulaveis
    inline static float velocidadeMinima = 8.0f;      // Km/h
    inline static float velocidadeMax = 20.0f;      // Km/h
    inline static float RPMideal = 3500.0f;
    inline static float PosBorboIdeal = 4000.0f;
    inline static int modoControle = 0;

   enum StatesStartStop {
        stateSwitchOFF            = 0,
        stateSwitchON             = 1,
        stateLigaMotor            = 2,
        stateDesligaMotor         = 3,
        stateEstabilizaAcelera    = 4,
        stateStart                = 5,
        stateStop                 = 6,
        stateFreando              = 7,
        stateDesligaStartStop     = 8,
        stateNotLigou             = 9,
        stateNotDesligou          = 10,
         
    };

    static StatesStartStop switchOFF ();

    static StatesStartStop switchON (); 


    static StatesStartStop ligaMotorSS (Motor &motor, Display &display);
    static StatesStartStop desligaMotorSS (Motor &motor, Display &display);
    static StatesStartStop estabilizaAcelera (Motor &motor);
    static StatesStartStop start (Motor &motor);
    static StatesStartStop stop (Motor &motor);
    static StatesStartStop freando ();
    static StatesStartStop notLigou (Display &display);
    static StatesStartStop notDesligou (Display &display);
    static StatesStartStop desligaStartStop (Motor &motor, Display &display);


static void Inicializar_sensores_startstop();

private:

    static int tentativasLigar;
    static int tentativasDesligar;
    
   // Mudar isso aqui abaixo não sei pq
    static unsigned long timerTentativa;
};

#endif
