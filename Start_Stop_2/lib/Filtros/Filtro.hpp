#pragma once
#include <Arduino.h>

class FiltroKalman {
private:
    float erro_medida;
    float erro_estimativa;
    float q;
    float estimativa_atual;
    bool primeira_leitura;

public:
    FiltroKalman(float k_mea, float k_est, float k_q);
    float aplicar(float valor_medido);
    void atualizarParametros(float k_mea, float k_est, float k_q);
};

class FiltroExponencial {
private:
    float alfa;
    float estimativa_atual;
    bool primeira_leitura;

public:
    FiltroExponencial(float fator_alfa);
    float aplicar(float valor_bruto);
};