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

    float getPosBorbo() { return posborbo;}

    float getLambda() {return lambda;}

    float getMap() { return map;}




    // 

 private:

    float posborbo = 0;
    float lambda = 0;
    float map = 0;
    bool primeiraLeituraTp = true;

};

#endif