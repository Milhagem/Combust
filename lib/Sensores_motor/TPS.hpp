#ifndef TPS_HPP
#define TPS_HPP
#include "Filtro_Exponencial.hpp"
#include <Arduino.h>

#define Pintp           6

class TPS {
 public:
    static float analisaPosBorbo();
    static float getPosBorbo() { return posborbo;}
    static bool return_status_tps(){return status_tps;}

 private:
    inline static FiltroExponencial filtroExponencialPosBorbo = FiltroExponencial(0.05f);
    inline static float posborbo = 0;
    inline static bool status_tps = false; // Essa e a proxima variavel me informa o status da chave de ignição
};

#endif