#pragma once
#include <Arduino.h>

class ServoMotor {
private:
    static constexpr uint8_t PIN_SERVO = 47; 
    inline static int pulsoMin = 500;
    inline static int pulsoMax = 2400;
    inline static int pulsoServo = 1056;

public:
    //Variavel manipulada remotamente
    inline static int posInicialServo = 1056;

    static void Start_servo();
    static void Escreve_servo(int microssegundos);
    
    // Métodos para o BSFC ler o estado atual do servo
    static int getPulsoAtual() { return pulsoServo; }
    static int getPosInicial() { return posInicialServo; }
};