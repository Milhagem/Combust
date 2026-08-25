#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include "TENSAO.hpp"

#define timeInterval 200 // ms

class Display {
 private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); 
    unsigned int timeOld = 0;
    bool displayConectado = false; 
    
    // Variáveis internas para o cronômetro do motor
    unsigned long timerMotorLigado = 0;
    bool motorEstavaLigado = false;
    float tempoTotalLigado = 0.0f;

    void autoReparo(); 

 public:
    void iniciaDisplay();
    
    // Assinatura MANTIDA para não quebrar a sua main.cpp
    void mostraTensaoEVel(float velocidade, float tensao);
    void atualizaDisplay(float velocidade, int FSMState, float tensao);
};

#endif