#include "BSFC.hpp"


    void BSFC::Escreve_servo(int microssegundos){

    if(microssegundos < 500) microssegundos = 500;
    if(microssegundos > 2500) microssegundos = 2500;
    uint32_t duty = (uint32_t)( ((uint64_t)microssegundos * 16384ULL) / 20000ULL );
    ledcWrite(PIN_SERVO, duty);
    }
  

    void BSFC::Start_servo(){
      pinMode(PIN_SERVO, OUTPUT);
      ledcSetup(0, 50, 14); 
      ledcAttachPin(PIN_SERVO, 0);
      pulsoServo = posInicialServo;
      Escreve_servo(pulsoServo);
    }

    void BSFC::Controle_RPM(float rpmAlvo, float rpmAtual, Sensores_motor::statesEngine estadoMotor, bool status_central) {
    static float erroAnterior = 0.0f;

    if (estadoMotor != Sensores_motor::engineOFF && status_central) {
        float erro = (rpmAlvo - rpmAtual) * 0.02f; 
        
        if (fabs(erro) > histerese) {
            float termoP = kpTun * (erro - erroAnterior);
            float termoI = kiTun * erro;                
            int compensacao = (int)(termoP + termoI);
            
            if (compensacao > passoMaxTun) compensacao = passoMaxTun;
            if (compensacao < -passoMaxTun) compensacao = -passoMaxTun;

            pulsoServo += compensacao;
            
            if (pulsoServo < pulsoMin) pulsoServo = pulsoMin;
            if (pulsoServo > pulsoMax) pulsoServo = pulsoMax;
        }
        erroAnterior = erro;
    } else {

        pulsoServo = posInicialServo; 
        erroAnterior = 0.0f;
    }
    Escreve_servo(pulsoServo);
}

    void BSFC::Controle_TPS(float tpsAlvo, float tpsAtual, Sensores_motor::statesEngine estadoMotor, bool status_central) {
    static float erroAnterior = 0.0f; 

    // Alteração mínima: mudamos de 'bool status_motor' para verificação do enum
    if (estadoMotor != Sensores_motor::engineOFF && status_central) {
        float erro = tpsAlvo - tpsAtual;
        
        if (fabs(erro) > histerese) {
            float termoP = kpTun * (erro - erroAnterior);
            float termoI = kiTun * erro;                
            int compensacao = (int)(termoP + termoI);
            
            if (compensacao > passoMaxTun) compensacao = passoMaxTun;
            if (compensacao < -passoMaxTun) compensacao = -passoMaxTun;

            pulsoServo += compensacao;
            
            if (pulsoServo < pulsoMin) pulsoServo = pulsoMin;
            if (pulsoServo > pulsoMax) pulsoServo = pulsoMax;
        }
        erroAnterior = erro;
    } else {
        pulsoServo = posInicialServo; 
        erroAnterior = 0.0f;
    }
    Escreve_servo(pulsoServo);
}


