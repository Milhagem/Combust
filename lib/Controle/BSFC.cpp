#include "BSFC.hpp"

    void BSFC::Controle_RPM(float rpmAlvo, float rpmAtual, Motor::statesEngine estadoMotor, bool status_central) {
        static unsigned long lastControleTime = 0;
        if (millis() - lastControleTime < 100) return; 
        lastControleTime = millis();
    
    
        static float erroAnterior = 0.0f;

    if (estadoMotor != Motor::engineOFF && status_central) {
        float erro = (rpmAlvo - rpmAtual) * 0.02f; 
        
        if (fabs(erro) > histerese) {
            float termoP = kpTun * (erro - erroAnterior);
            float termoI = kiTun * erro;                
            int compensacao = (int)(termoP + termoI);
            if (compensacao > passoMaxTun) { compensacao = passoMaxTun; }
            if (compensacao < -passoMaxTun) { compensacao = -passoMaxTun; }

            int novoPulso = ServoMotor::getPulsoAtual() + compensacao;
            ServoMotor::Escreve_servo(novoPulso);}
        erroAnterior = erro;
    } else { 
        erroAnterior = 0.0f;
        ServoMotor::Escreve_servo(ServoMotor::getPosInicial());
    }
}

    void BSFC::Controle_TPS(float tpsAlvo, float tpsAtual, Motor::statesEngine estadoMotor, bool status_central) {
        static unsigned long lastControleTimetps = 0;
        if (millis() - lastControleTimetps < 100) return;
        lastControleTimetps = millis();
    
    
        static float erroAnterior = 0.0f; 

    // Alteração mínima: mudamos de 'bool status_motor' para verificação do enum
    if (estadoMotor != Motor::engineOFF && status_central) {
        float erro = tpsAlvo - tpsAtual;
        
        if (fabs(erro) > histerese) {
            float termoP = kpTun * (erro - erroAnterior);
            float termoI = kiTun * erro;                
            int compensacao = (int)(termoP + termoI);
            if (compensacao > passoMaxTun) compensacao = passoMaxTun;
            if (compensacao < -passoMaxTun) compensacao = -passoMaxTun;

            int novoPulso = ServoMotor::getPulsoAtual() + compensacao;
            ServoMotor::Escreve_servo(novoPulso);}
        erroAnterior = erro;
    } else {
        erroAnterior = 0.0f;
        ServoMotor::Escreve_servo(ServoMotor::getPosInicial());
}
    }





