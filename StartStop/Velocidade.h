#ifndef VELOCIDADE_H
#define VELOCIDADE_H

#define pinSensorHall      DD2
#define pinPedalGND        A8  // Pino p/ simulacao de velocidade em testes em bancada
#define pinPedalVCC        A9  // Pino p/ simulacao de velocidade em testes em bancada
#define pinVelPedal        A10 // Pino p/ simulacao de velocidade em testes em bancada

#define taxaAtualizacaoVel 1000  // ms
#define diametroRodaURban  0.015 // m
#define quantImas          5

#define MPS_to_KMPH   3.6 // metros por segundo p/ quilometro por hora
#define MS_to_S     000.1 // milisegundos p/ segundos

int velocidadeAtual;          // km/h
unsigned long lastmillis;     // ms
volatile int picoLeituraHall;

/**
 * @brief Atualiza a velocidade com base nos pulsos do Sensor Hall
 * 
 * @details Usa a interrupcao do Arduino para computar a quantidade de pulsos em um intervalo de tempo. Retorna a velocidade atualiza OU a velocidade nao atualizada (passada como parametro).
 * 
 * @param velocidadeAtual velocidade antes da chamada da funcao. Sera retornada caso a velocidade nao seja atualizada.
 * 
 * @return velocidade (m/s)
 */
int atualizaVelocidadeAtual(int velocidadeAtual);
void variador () { picoLeituraHall++; }


#endif