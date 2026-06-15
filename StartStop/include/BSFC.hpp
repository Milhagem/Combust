#ifndef BSFC_HPP
#define BSFC_HPP

#include <Arduino.h>
#include <SimpleKalmanFilter.h> 

#define pinmap          3
#define pin02           2  
#define Pintp           6
#define SensorCKP       37 

class Bsfc {
 public:
    // Inicializa o filtro com valores padrão que serão sobrescritos depois
    Bsfc() : filtroKalmanRPM(20.0, 20.0, 0.5) {}

    void init(); 
    float analisaRPM();
    float analisaPosBorbo();
    float analisaMap();
    float analisaLambda();
    
    float getPosBorbo() { return posborbo;}
    float getLambda() {return lambda;}
    float getMap() { return map;}
    float getRpm() { return rpm;}

    // Função para receber os parâmetros do MQTT sem reiniciar
    void atualizaKalman(float mea_e, float est_e, float q);

    static volatile unsigned long tempoAnterior;
    static volatile unsigned long rpm_calculado; 

 private:
    float rpm = 0;
    float posborbo = 0;
    float lambda = 0;
    float map = 0;
    bool primeiraLeituraTp = true;
    
    static volatile unsigned long hw_deltaTAnterior;
    static int hw_contadorDentes;
    static int hw_dentesAcumulados;
    static volatile unsigned long hw_tempoUltimaVolta;
    static volatile unsigned long hw_tempoAcumulado;

    static void IRAM_ATTR lerCKP();

    // Objeto do filtro encapsulado na classe
    SimpleKalmanFilter filtroKalmanRPM; 
};

#endif
