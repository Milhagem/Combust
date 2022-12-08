#ifndef DISPLAY_H
#define DISPLAY_H

#include "Wire.h"
#include "LiquidCrystal_I2C.h"
#include "Motor.h"

#define timeInterval 200 // ms

class Display {
    private:
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);
    unsigned int timeOld;

    public:
    /**
     * @brief Esta funcao tem o unico objetivo de inicializar o Display LDC no setup() StartStop.ino
    */
    void iniciaDisplay();
    /**
     * Atualiza o Display com a tensao e velocidade. Taxa de atualizacao = timeInterval
     * 
     * @param motor Motor cujos dados serao lidos
    */
    void atualizaDisplay(Motor &motor);


};

#endif