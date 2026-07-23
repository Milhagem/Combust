#include "Tensao.hpp" 

float Tensao::analisaTensao(){
  const float RESOLUCAO_ESP = 4095.0f;
  const float TENSAO_ESP = 3.3f;

  float valorInicial = analogRead(LM2907);
  float tensao_bruta = (valorInicial/RESOLUCAO_ESP)*TENSAO_ESP; 
  tensao = filtroExponencialTensao.aplicar(tensao_bruta);
  return tensao;
}