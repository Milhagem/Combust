#include "MAP.hpp" 

float MAP::analisaMap(){
  const float valorMin = 100.0f;
  const float valorMax = 4095.0f;

  float valorInicial = analogRead(pinMAP);
  
  if (valorInicial < valorMin){
    valorInicial = valorMin;
    status_map = false;
  } else {
    status_map = true;
  }

  if (valorInicial > valorMax) {
    valorInicial = valorMax;
  }
  
  float porcentagem = ((valorInicial-valorMin)/(valorMax-valorMin)) * 100.0f;
  map = porcentagem;

  return porcentagem;
}