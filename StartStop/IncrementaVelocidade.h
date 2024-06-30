#ifndef INCREMENTAVELOCIDADE_H
#define INCREMENTAVELOCIDADE_H

#include <Arduino.h>
#include "Motor.h"

#define pinServo   8

#define posZeroServo          0 // graus. Posicao que o servo NAO puxa a borboleta
#define posInicialServo      40 // graus. Posicao em que o servo comeca a puxar a borboleta
#define posMaxServo         170 // graus
#define increvementoServo     1 // graus
#define intervIncrementaVel 200 // ms

/**
 * @brief Acelera o motor alterando o angulo do servo.
 * 
 * @details Inicialmente, o é checado se o motor foi acelerado. Isso eh necessario pois ha uma folga no cabo de aco que o servo motor usa para puxar a borboleta. Essa folga eh corrigida colocando a posicao do servo em um angulo pre determinado.
 * 
 * Feito isso, a cada intervalo de tempo, o servo motor muda um pouco o angulo ate que a velocidade maxima seja atingida.
 * 
 * @param Motor objeto motor usado no codigo
 */
void funcIncrementaVel(Motor &Motor);

#endif

