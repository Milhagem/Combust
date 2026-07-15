#ifndef SENSOR_motor_HPP
#define SENSOR_motor_HPP
#include "Filtro.hpp"
#include <Arduino.h>

#define pinmap          3
#define pin02           2  
#define Pintp           6
#define LM2907          10
#define SensorCKP       37

#define tensaoMotorON         0.67

class Sensores_motor {
 public:

    enum statesEngine { engineOFF, engineON, accelerating };
    bool status_central = false;

   
    void Inicializar_setup_sensores_motor(); 
    float analisaRPM();
    float analisaPosBorbo();
    float analisaMap();
    float analisaLambda();
    float analisaTensao();
    void analisa_status_central();
    statesEngine analisa_status_motor();


    
    
    float getPosBorbo() { return posborbo;}
    float getLambda() {return lambda;}
    float getMap() { return map;}
    float getRpm() { return rpm;}
    bool getStatusCentral() { return status_central; }
    static bool getStatusMotor() { return status_motor; } 


 private:

    FiltroExponencial filtroExponencialPosBorbo = FiltroExponencial(0.05f);
    FiltroExponencial filtroExponencialLambda = FiltroExponencial(0.1f);
    FiltroExponencial filtroExponencialTensao = FiltroExponencial(0.5f);
    FiltroKalman filtroKalmanRPM = FiltroKalman(35.0f, 10.0f, 0.02f);

    float rpm = 0;
    float posborbo = 0;
    float lambda = 0;
    float map = 0;
    float tensao = 0;
    bool status_tps = false; // Essa e a proxima variavel me informa o status da chave de ignição
    bool status_map = false;
        
    inline static volatile unsigned long tempo_central_desligada = 0;   
    inline static volatile bool status_motor = false;
   
    
    inline static volatile unsigned long tempoAnterior = 0;
    inline static volatile unsigned long rpm_calculado = 0;
    inline static volatile unsigned long hw_deltaTAnterior = 0;
    inline static int hw_contadorDentes = 0;
    inline static int hw_dentesAcumulados = 0;
    inline static volatile unsigned long hw_tempoUltimaVolta = 0;
    inline static volatile unsigned long hw_tempoAcumulado = 0;
    
    static void IRAM_ATTR lerCKP();

};

#endif 