#pragma once

#include <Arduino.h>

class Filtro_Kalman {
private:
    float erro_medida;
    float erro_estimativa;
    float q;
    float estimativa_atual;
    bool primeira_leitura;

public:
    Filtro_Kalman(float k_mea, float k_est, float k_q);
    void atualizarParametros(float k_mea, float k_est, float k_q);
    float aplicar(float valor_medido);
};
