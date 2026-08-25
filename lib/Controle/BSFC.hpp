#pragma once
#include <Arduino.h>
#include "Ckp.hpp"
#include "Lambda.hpp"
#include "Map.hpp"
#include "TPS.hpp"
#include "Tensao.hpp"
#include "Motor.hpp"
#include "Servo.hpp"

class BSFC{
  private:  

   
    inline static const float histerese = 0.6f;
    
  public:

    inline static float kpTun = 0.0f;       
    inline static float kiTun = 0.8f;       
    inline static int passoMaxTun = 30;
    
    static void Controle_RPM(float rpmAlvo, float rpmAtual, Motor::statesEngine, bool status_central);
    static void Controle_TPS(float tpsAlvo, float tpsAtual, Motor::statesEngine, bool status_central);
  };