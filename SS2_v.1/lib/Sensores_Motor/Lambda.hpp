#ifndef LAMBDA_HPP
#define LAMBDA_HPP
#include "Filtro_Exponencial.hpp"
#include <Arduino.h>

#define pin02           2  

class Lambda {
 public:
    static float analisaLambda();
    static float getLambda() {return lambda;}

 private:
    inline static FiltroExponencial filtroExponencialLambda = FiltroExponencial(0.1f);
    inline static float lambda = 0;
};

#endif