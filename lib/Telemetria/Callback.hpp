#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <PubSubClient.h>


#include "StartStop.hpp"
#include "Motor.hpp"
#include "Servo.hpp"
#include "BSFC.hpp" 

class Callback {
public:
    static void carregarParametrosIniciais();
    static void processarMensagem(byte* payload, unsigned int length, PubSubClient& client, const char* topico_feedback);
};