#ifndef LAMBDA_HPP
#define LAMBDA_HPP

#include <Arduino.h>

#include "Filtro_Exponencial.hpp"

#define pinLambda           2

class Lambda {
 public:
    static float analisaLambda();
    static float getLambda() { return lambda; }

 private:
    inline static Filtro_Exponencial filtroExponencialLambda = Filtro_Exponencial(0.1f);
    inline static float lambda = 0;
};

#endif
