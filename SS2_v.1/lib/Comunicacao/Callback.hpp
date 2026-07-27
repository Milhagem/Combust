#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <PubSubClient.h>

#include "Motor.hpp"
#include "Servo.hpp"
#include "StartStop.hpp"

class Callback {
public:
    static void carregarParametrosIniciais();
    static void processarMensagem(byte* payload, unsigned int length, PubSubClient& client, const char* topico_feedback);
};
