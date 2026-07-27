#include "Filtro_Exponencial.hpp"

//----------------------------------------------------------
// Implementação do Filtro Exponencial
//----------------------------------------------------------
Filtro_Exponencial::Filtro_Exponencial(float fator_alfa) {
    alfa = fator_alfa;
    estimativa_atual = 0.0;
    primeira_leitura = true; 
}

float Filtro_Exponencial::aplicar(float valor_bruto) {
    if (primeira_leitura) {
        estimativa_atual = valor_bruto;
        primeira_leitura = false;
    } else {
        estimativa_atual = (alfa * valor_bruto) + ((1.0f - alfa) * estimativa_atual);
    }
    
    return estimativa_atual;
}