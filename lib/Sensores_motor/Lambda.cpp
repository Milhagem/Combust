#include "Lambda.hpp" 

float Lambda::analisaLambda(){
  const float valorVMin = 140.0f;
  const float valorVMax = 4000.0f;
  const float valorLambdaMin = 0.5f;
  const float ValorLambdaMax = 2.8f;
  float valorInicial = analogRead(pin02);
  float Lambda_bruto = 0.0f;

  
  Lambda_bruto = valorLambdaMin + ((valorInicial-valorVMin)/(valorVMax-valorVMin))*(ValorLambdaMax-valorLambdaMin);
  
  if (Lambda_bruto < valorLambdaMin) Lambda_bruto = valorLambdaMin;
  if (Lambda_bruto > ValorLambdaMax) Lambda_bruto = ValorLambdaMax;
  
  lambda = filtroExponencialLambda.aplicar(Lambda_bruto);
  return lambda;
}