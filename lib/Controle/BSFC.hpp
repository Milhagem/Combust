#pragma once
#include <Arduino.h>
#include "Sensores_motor.hpp"

class BSFC{
  private:  
    static constexpr uint8_t PIN_SERVO = 47; 
    inline static float kpTun = 0.0f;       
    inline static float kiTun = 4.5f;       
    inline static int passoMaxTun = 31;
    inline static int posInicialServo = 1056;

    inline static long servoInterval = 100;
    inline static int pulsoMin = 500;
    inline static int pulsoMax = 2400;
    inline static int pulsoServo = 1056;
    inline static const float histerese = 0.6f;
    inline static unsigned long lastServoTime = 0;
    
  public:
    static void Escreve_servo(int microssegundos);
    static void Start_servo();
    static void Controle_RPM(float rpmAlvo, float rpmAtual, Sensores_motor::statesEngine estadoMotor, bool status_central);
    static void Controle_TPS(float tpsAlvo, float tpsAtual, Sensores_motor::statesEngine estadoMotor, bool status_central);
  };