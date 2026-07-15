#pragma once 
#include "Velocidade.hpp"
#include "Sensores_motor.hpp" 
#include "BSFC.hpp"
#include <Arduino.h>

class Display;

class Motor {
public:
    static void Parametros_setup_controle_motor();
    static Sensores_motor::statesEngine ligaMotor(Display& display, Sensores_motor& sensores);
    static Sensores_motor::statesEngine desligaMotor(Display& display, Sensores_motor& sensores);

private:
    static constexpr uint8_t PIN_LIGA_MOTOR = 10;
    static constexpr uint8_t PIN_DESLIGA_MOTOR = 11;
    static constexpr int POS_SERVO_PARTIDA = 1000;
    static constexpr int POS_SERVO_FECHADA = 500;
    
    inline static int posServoAtual = POS_SERVO_FECHADA;
};