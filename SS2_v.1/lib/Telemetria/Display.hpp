#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include "TENSAO.hpp" // Certifique-se de que o arquivo físico está em MAIÚSCULO, senão mude para "Tensao.hpp"

#define timeInterval 200 // ms

class Display {
    private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2); 
    unsigned int timeOld;

    public:
    void iniciaDisplay();
    void mostraTensaoEVel(float velocidade, float tensao);
    void atualizaDisplay(float velocidade, int FSMState, float tensao);
};

#endif