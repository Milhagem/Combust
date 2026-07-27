#pragma once

#include <Arduino.h>

class Filtro_Exponencial {
private:
    float alfa;
    float estimativa_atual;
    bool primeira_leitura;

public:
    Filtro_Exponencial(float fator_alfa);
    float aplicar(float valor_bruto);
};
