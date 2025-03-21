#ifndef DISPLAY_H
#define DISPLAY_H

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include "Motor.hpp"
//#include "StartStop.hpp"   


#define timeInterval 200 // ms

class Display {
    private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);
    unsigned int timeOld;

    public:

    //Display();
    /**
     * @brief Esta funcao tem o unico objetivo de inicializar o Display LDC no setup() StartStop.ino do Arduino
    */
    void iniciaDisplay();
    /**
     * @brief Atualiza o Display com a velocidade e tensao. Taxa de atualizacao = timeInterval
     * 
     * @param motor Motor cujos dados serao lidos
     * @param speed Velocidade a ser exibida
    */
    void mostraTensaoEVel(float velocidade);
    /**
     * @brief Atualiza o Display com a velocidade e FSM. Taxa de atualizacao = timeInterval
     * 
     * @param motor Motor cujos dados serao lidos
     * @param speed Velocidade a ser exibida
     * @param FSMState_int Numero correspondente ao estado da Maquina de Estados
    */
    void atualizaDisplay ( float velocidade, int FSMState);

};

#endif