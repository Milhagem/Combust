#include "BSFC.hpp"

// ++++++++++++++++   Essa parte será mudada para encontrar o rpm do Bsfc******    ->    return rpm FILTRO DE KALMAN?

float Bsfc::analisaRPM(){                   
  float rpm=1000;
  return rpm;
}

//+++++++++++++++++++

float Bsfc::analisaPosBorbo(){   //pega a informação se linear e passa para porcentagem
  const float valorMin=100.0;
  const float valorMax=1023.0;

  const float ALFA = 0.3; // ultilizando uma media para filtrar variações( as mais recentes tem mais peso)


  float leituraAtual = analogRead(Pintp);

  if (leituraAtual < valorMin) leituraAtual = valorMin;
  if (leituraAtual > valorMax) leituraAtual = valorMax;

  float porcentagemAtual = ((leituraAtual - valorMin) / (valorMax - valorMin)) * 100.0;

 if (primeiraLeituraTp) {
    posborbo = porcentagemAtual;
    primeiraLeituraTp = false;
  } else {
    posborbo = (ALFA * porcentagemAtual) + ((1.0 - ALFA) * posborbo);
  }

  return posborbo;
}


float Bsfc::analisaMap(){
  const float RESOLUCAO_ARDUINO = 1024.0;
  const float valorMin = 100.0;
  const float valorMax = 1024.0;
  float valorInicial = analogRead(pinmap);

  if (valorInicial < valorMin) valorInicial = valorMin;
  if (valorInicial > valorMax) valorInicial = valorMax;

  float porcentagem = ((valorInicial-valorMin)/(valorMax-valorMin)) * 100;
  map=porcentagem;
  return porcentagem; // no futuro passar para pressão???
}


float Bsfc::analisaLambda(){ // verificar a precisão da leitura, fazer media??
  const float RESOLUCAO_ARDUINO = 1024.0;
  const float valorVMin = 100.0;  // Relação da tensão maxima dada no manual(0.2v= algum valor no arduino ex:70)
  const float valorVMax = 1024.0;  // Relação da tensão maxima dada no manual(4.8v= algum valor no arduino ex:950)
  const float valorLambdaMin = 0.5;  // No manual tem as escalas dadas
  const float ValorLambdaMax = 2.8;   // No manual tem as escalas dadas

  float valorInicial = analogRead(pin02);
  lambda = valorLambdaMin + ((valorInicial-valorVMin)/(valorVMax-valorVMin))*(ValorLambdaMax-valorLambdaMin);

  if (lambda < valorLambdaMin) lambda = valorLambdaMin;
  if (lambda > ValorLambdaMax) lambda = ValorLambdaMax;
  return lambda;
}