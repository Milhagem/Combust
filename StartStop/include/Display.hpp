#ifndef DISPLAY_H
#define DISPLAY_H

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include "Motor.hpp"
#include "StartStop.hpp"   

class Motor;

#define timeInterval 200 // ms

class Display {
    private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);
    unsigned int timeOld;

    public:

    void iniciaDisplay();
    
    void mostraTensaoEVel(float velocidade, Motor &motor);
  
    void atualizaDisplay ( float velocidade, int FSMState, Motor &motor);

};

#endif