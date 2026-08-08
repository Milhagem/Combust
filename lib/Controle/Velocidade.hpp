#pragma once
#include <Arduino.h>
#include "Filtro_Kalman.hpp"

class Velocidade {
public:
    static void Inicializar_setup_sensores_velocidade(); 
    static void setVelocidade (float &vel);
    static float getVelocidade ();
    
    static void setAceleracao (float &aceleracao); // Corrigido: adicionado static
    static float getAcelera ();
    static float getRPM ();
    
    static void IRAM_ATTR calc();
    static float calculaVelocidade();
    static float filtroVelocVariacoesGrandes(float velocidadeOld, float velocidadeNew); // Corrigido: adicionado static
    static void incrementaPulsos();

private:
    // ========================================================
    // CONSTANTES DE HARDWARE E FÍSICA
    // ========================================================
    static constexpr uint8_t PIN_SENSOR_HALL = 1; // Corrigido: Trazido para dentro da classe
    
    static constexpr uint16_t TAXA_ATUALIZACAO_VEL = 200; // ms
    static constexpr uint8_t PULSOS_POR_VOLTA = 5;
    static constexpr uint8_t SAMPLE_SIZE = 2;

    static constexpr float CIRCUNF_RODA = 1.81f;        // m
    static constexpr float MPS_TO_KMPH_FACTOR = 3.6f;
    
    // ========================================================
    // VARIÁVEIS DO SISTEMA
    // ========================================================
    inline static float velocidade = 0.0f;
    inline static float acelera = 0.0f;
    inline static float RPM = 0.0f;
    inline static unsigned long pulsos = 0;

    inline static volatile uint64_t pulseInterval = 0; 
    inline static volatile uint64_t lastPulseInterval = 0; 
    inline static volatile uint64_t pulseIntervals[SAMPLE_SIZE] = {0};
    inline static volatile int pulseIndex = 0;
    
    inline static unsigned long lastTimerTax = 0;  
    inline static float velocOld = 0.0f;

    inline static FiltroKalman filtro_Kalman_Vel{10.0f, 5.0f, 0.5f};
    inline static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};