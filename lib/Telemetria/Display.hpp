#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include "TENSAO.hpp"

#define timeInterval 200 // ms

class Display {
    private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2); 
    unsigned int timeOld = 0;
    
    // Variável para saber se a tela está respondendo
    bool displayConectado = false; 
    
    // Função mágica que concerta o mau contato
    void autoReparo(); 

    public:
    void iniciaDisplay();
    void mostraTensaoEVel(float velocidade, float tensao);
    void atualizaDisplay(float velocidade, int FSMState, float tensao);
};

#endif