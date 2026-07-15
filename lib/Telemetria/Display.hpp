#ifndef DISPLAY_H
#define DISPLAY_H

#include "Motor.hpp"
#include "StartStop.hpp"   
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

class Motor;

#define timeInterval 200 // ms

class Display {
    private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2); // Endereço, número de colunas, número de linhas
    
    unsigned int timeOld;

    public:
    void iniciaDisplay();
    
    void mostraTensaoEVel(float velocidade, Sensores_motor &sensores);
  
    void atualizaDisplay(float velocidade, int FSMState, Sensores_motor &sensores);
};

#endif