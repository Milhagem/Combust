#include "TPS.hpp" 

float TPS::analisaPosBorbo() {   
  const float valorMin = 450.0f;
  const float valorMax = 3500.0f;
  
  float leituraAtual = analogRead(PinTPS);

  if (leituraAtual < valorMin) {
    leituraAtual = valorMin;
    status_tps = false;
  } else {
    status_tps = true;
  }
  if (leituraAtual > valorMax) leituraAtual = valorMax;

  float porcentagemAtual = ((leituraAtual - valorMin) / (valorMax - valorMin)) * 100.0f;

  posborbo = filtroExponencialPosBorbo.aplicar(porcentagemAtual);
  
  return posborbo;
}