#pragma once 

#include "CKP.hpp"
#include "Display.hpp"
#include "Hall.hpp"
#include "Lambda.hpp"
#include "LM2907.hpp"
#include "MAP.hpp"
#include "Servo.hpp"
#include "TPS.hpp"

#include <Arduino.h>

class Display;

class Motor {
public:
    inline static unsigned long timerPartida = 0;
    inline static bool acionando = false;
    enum statesEngine { engineOFF, engineON, accelerating };

    static void Parametros_setup_controle_e_sensores_motor();
    
    static statesEngine ligaMotor(Display& display);
    static statesEngine desligaMotor(Display& display);

    static void analisa_status_central();
    static statesEngine analisa_status_motor();
    static void analisa_sensores_motor();
    static bool getStatusCentral() { return status_central; }
    inline static int POS_SERVO_PARTIDA = 1000;
    inline static int POS_SERVO_FECHADA = 500;

private:
    static constexpr uint8_t PIN_LIGA_MOTOR = 10;
    static constexpr uint8_t PIN_DESLIGA_MOTOR = 11;

    inline static int posServoAtual = POS_SERVO_FECHADA;

    inline static bool status_central = false;
    inline static unsigned long tempo_central_desligada = 0;   
};
