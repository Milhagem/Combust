#include "Filtro_Exponencial.hpp"

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