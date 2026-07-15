#include "Filtro.hpp"


//----------------------------------------------------------
// Implementação do Filtro de Kalman
//----------------------------------------------------------

FiltroKalman::FiltroKalman(float k_mea, float k_est, float k_q) {
    erro_medida = k_mea;
    erro_estimativa = k_est;
    q = k_q;
    estimativa_atual = 0.0; 
}

float FiltroKalman::aplicar(float valor_medido) {
    if (primeira_leitura) {
        estimativa_atual = valor_medido;
        primeira_leitura = false;
    } else {
        erro_estimativa = erro_estimativa + q;
        
        // A LINHA ABAIXO PRECISA ESTAR AQUI (com o tipo float antes):
        float ganho_kalman = erro_estimativa / (erro_estimativa + erro_medida);
        
        estimativa_atual = estimativa_atual + ganho_kalman * (valor_medido - estimativa_atual);
        erro_estimativa = (1.0f - ganho_kalman) * erro_estimativa; 
    }
    return estimativa_atual;
}

void FiltroKalman::atualizarParametros(float k_mea, float k_est, float k_q) {
    erro_medida = k_mea;
    erro_estimativa = k_est;
    q = k_q;
}


//----------------------------------------------------------
// Implementação do Filtro Exponencial
//----------------------------------------------------------
FiltroExponencial::FiltroExponencial(float fator_alfa) {
    alfa = fator_alfa;
    estimativa_atual = 0.0;
    primeira_leitura = true; // Resolve aquele seu 'if' de primeira leitura!
}

float FiltroExponencial::aplicar(float valor_bruto) {
    if (primeira_leitura) {
        estimativa_atual = valor_bruto;
        primeira_leitura = false;
    } else {
        // A mesma matemática limpa que você já usava
        estimativa_atual = (alfa * valor_bruto) + ((1.0f - alfa) * estimativa_atual);
    }
    return estimativa_atual;
}