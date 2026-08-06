#ifndef CKP_HPP
#define CKP_HPP
#include "Filtro_Kalman.hpp"
#include <Arduino.h>

#define SensorCKP       37

class Ckp {
 public:
    static void Inicializar_setup_sensores_motor(); 
    static float analisaRPM();
    static float getRpm() { return rpm;}
    static bool getStatusMotor() { return status_motor; } 

 private:
    inline static FiltroKalman filtroKalmanRPM = FiltroKalman(20.0f, 20.0f, 0.25f);
    inline static float rpm = 0;

    inline static volatile bool status_motor = false;
   
    inline static volatile unsigned long tempoAnterior = 0;
    inline static volatile unsigned long rpm_calculado = 0;
    inline static volatile unsigned long hw_deltaTAnterior = 0;
    inline static volatile int hw_contadorDentes = 0;
    inline static volatile int hw_dentesAcumulados = 0;
    inline static volatile unsigned long hw_tempoUltimaVolta = 0;
    inline static volatile unsigned long hw_tempoAcumulado = 0;
    
    static void IRAM_ATTR lerCKP();
};

#endif