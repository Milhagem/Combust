#ifndef VELOCIDADE_H
#define VELOCIDADE_H

#include "Arduino.h"

#define pinSensorHall      DD2
#define pinPedalGND        A8  // Pino p/ simulacao de velocidade em testes em bancada
#define pinPedalVCC        A9  // Pino p/ simulacao de velocidade em testes em bancada
#define pinVelPedal        A10 // Pino p/ simulacao de velocidade em testes em bancada

#define velZERO            150  // Valores para teste
#define velMin             400  // Valores para teste
#define velMax             700  // Valores para teste
#define taxaAtualizacaoVel 1000 // ms
#define circunfRoda        0.47 // m
#define pulsosPorVolta     5
#define sampleSize         3    // Numero de amostras para calcular velocidade

#define MPS_to_KMPH 3.6  // metros por segundo p/ quilometro por hora
#define MS_to_S     1000 // milisegundos p/ segundos

/** 
 * @brief Armazena o valor de periodo da onda no vetor usado para o calculo de velocidade
 * 
 */
void calc();

/**
 * @brief Atualiza a velocidade com base nos pulsos do Sensor Hall 
 * 
 * @details DEFINIR
 * 
 * @param velocidadeAtual velocidade antes da chamada da funcao.
 * 
 * @return velocidade (m/s)
 */
float calculaVelocidade(float speed);

/**
 * @brief Filtra variacoes absuradas de velocidade
 * 
 * @param velocidadeRaw 
 * @return velocidade (km/h) 
 */
float filtroVelocidade(float velocidade, float velocidadeNew);

/**
 * @brief retorna um valor de velocidade com base na leitura de um pino analogico para testes em bancada
 * 
 * @param pinPedal 
 * @return int entre 0 e 1023
 */
int calculaVelocidade_Pedal(int pinPedal);

#endif