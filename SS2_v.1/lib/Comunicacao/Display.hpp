#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include "LM2907.hpp"

#define timeInterval 200

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
