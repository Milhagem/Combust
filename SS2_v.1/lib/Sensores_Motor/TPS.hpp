#ifndef TPS_HPP
#define TPS_HPP

#include "Filtro_Exponencial.hpp"

#include <Arduino.h>

#define PinTPS          6

class TPS {
 public:
    static float analisaPosBorbo();
    static float getPosBorbo() { return posborbo;}
    static bool return_status_tps(){return status_tps;}

 private:
    inline static Filtro_Exponencial filtroExponencialPosBorbo = Filtro_Exponencial(0.05f);
    inline static float posborbo = 0;
    inline static bool status_tps = false; // Essa e a próxima variável me informam o status da chave de ignição
};

#endif
