#ifndef TENSAO_HPP
#define TENSAO_HPP
#include "Filtro_Exponencial.hpp"
#include <Arduino.h>

#define LM2907          5
#define tensaoMotorON   0.67

class Tensao {
 public:
    static float analisaTensao();
    static float getTensao() { return tensao; }

 private:
    inline static FiltroExponencial filtroExponencialTensao = FiltroExponencial(0.7f);
    inline static float tensao = 0;
};

#endif