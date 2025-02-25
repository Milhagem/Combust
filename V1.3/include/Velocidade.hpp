#ifndef VELOCIDADE_H
#define VELOCIDADE_H

#include "Arduino.h"

#define pinSensorHall      DD2
// #define pinPedalGND        A8  // Pino p/ simulacao de velocidade em testes em bancada
// #define pinPedalVCC        A9  // Pino p/ simulacao de velocidade em testes em bancada
// #define pinVelPedal        A10 // Pino p/ simulacao de velocidade em testes em bancada

#define velZERO            0  // Valores para teste
#define velMin             6  // Valores para teste
#define velMax             15  // Valores para teste
#define taxaAtualizacaoVel 1000 // ms
#define circunfRoda        0.37 // m
#define pulsosPorVolta     1
#define sampleSize         3    // Numero de amostras para calcular velocidade

#define MPS_to_KMPH 3.6  // metros por segundo p/ quilometro por hora
#define MS_to_S     1000 // milisegundos p/ segundos


class Velocidade {
public:

    static void setVelocidade (float &vel);

    static float getVelocidade ();

    static void setAceleração (float &acel);

    static float getAceleração ();
    /** 
     * @brief Calcula o periodo dos pulsos do sensor Hall
     * 
     */
    static void calc();


    /**
     * @brief Atualiza a velocidade com base nos pulsos do Sensor Hall 
     * 
     * @details Usa amostras do PERIODO dos pulsos do Sensor Hall p/ calculo da velocidade. Sao aplicados 2 filtros: 
     * 1 - Media da amostra dos periodos; 
     * 2 - Corte de alteracoes bruscas.  no calculo dessa 
     * Essa funcao altera variavel passada como parametro (variavel global velocidade)
     * 
     * @param veloc velocidade antes da chamada da funcao.
     * 
     * @return velocidade (m/s)
     */
    static float calculaVelocidade();

    /**
     * @brief Filtra variacoes absurdas de velocidade
     * @brief Filtra variacoes absurdas de velocidade
     * 
     * @param velocidadeOld velocidade antes da atualizacao
     * @param velocidadeNew possivel novo valor de velocidada
     * @return velocidade (km/h) 
     */
    float filtroVelocVariacoesGrandes(float velocidadeOld, float velocidadeNew);

    /**
     * @brief retorna um valor de velocidade com base na leitura de um pino analogico para testes em bancada
     * 
     * @param pinPedal 
     * @return int entre 0 e 1023
     */
    int calculaVelocidade_Pedal(int pinPedal);

private:
    static float velocidade;
    static float aceleração;
};
#endif
