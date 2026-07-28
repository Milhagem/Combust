#pragma once

#include "CKP.hpp"
#include "Lambda.hpp"
#include "LM2907.hpp"
#include "MAP.hpp"
#include "Motor.hpp"
#include "TPS.hpp"
#include "Servo.hpp"

#include <Arduino.h>

class BSFC {
private:  
  inline static float kpTun = 0.0f;       
  inline static float kiTun = 4.5f;       
  inline static int passoMaxTun = 31;
 
  inline static const float histerese = 0.6f;
    
public:
  static void Controle_RPM(float rpmAlvo, float rpmAtual, Motor::statesEngine, bool status_central);
  static void Controle_TPS(float tpsAlvo, float tpsAtual, Motor::statesEngine, bool status_central);
};
