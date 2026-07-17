#pragma once
#include <Arduino.h>

class FiltroExponencial {
private:
    float alfa;
    float estimativa_atual;
    bool primeira_leitura;

public:
    FiltroExponencial(float fator_alfa);
    float aplicar(float valor_bruto);
};