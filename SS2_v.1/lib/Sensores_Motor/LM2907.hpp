#ifndef LM2907_HPP
#define LM2907_HPP

#include "Filtro_Exponencial.hpp"

#include <Arduino.h>

#define pinLM2907       5
#define tensaoMotorON   0.67

class LM2907 {
public:
   static float analisaTensao();
   static float getTensao() { return tensao; }

private:
   inline static Filtro_Exponencial filtroExponencialTensao = Filtro_Exponencial(0.7f);
   inline static float tensao = 0;
};

#endif
