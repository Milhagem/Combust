#ifndef BSFC_H
#define BSFC_H


#define pinServo        8

//pinos que precisam ser definidos
#define pinmap          a
#define pin02           b    
#define Pintp           c
#define SensorCKP       A2

class Bsfc {

 public:

    float analisaRPM();
    
    float analisaPosBorbo();

    float analisaMap();

    float analisaLambda();

     aceleraEficiente(); //calcula a quantidade de combustível a ser injetada com base nos dados coletados e na curva de consumo específico do motor
    


   //Futuras telemetria
    float getPosBorbo() { return posborbo;}

    float getLambda() {return lambda;}

    float getMap() { return map;}

    float getRpm() { return rpm;}




    // 

 private:
    float rpm = 0;
    float posborbo = 0;
    float lambda = 0;
    float map = 0;
    bool primeiraLeituraTp = true;

};

#endif
