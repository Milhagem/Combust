#include "Servo.hpp"

// Criamos uma constante para o canal para não ter confusão
const int CANAL_PWM = 0; 

void Servo::Start_servo() {
    // 1. Configura o gerador de PWM no Canal 0 (50Hz, 14 bits)
    ledcSetup(CANAL_PWM, 50, 14); 
    
    // 2. Conecta o Pino 47 físico ao Canal 0 virtual
    ledcAttachPin(PIN_SERVO, CANAL_PWM);

    pulsoServo = posInicialServo;
    Escreve_servo(pulsoServo);
}

void Servo::Escreve_servo(int microssegundos) {
   
    if(microssegundos < pulsoMin) microssegundos = pulsoMin;
    if(microssegundos > pulsoMax) microssegundos = pulsoMax; 
    
    pulsoServo = microssegundos; // Atualiza a variável de estado interna

    uint32_t duty = (uint32_t)( ((uint64_t)microssegundos * 16384ULL) / 20000ULL );
    
    // 3. A CORREÇÃO: Enviamos o pulso para o CANAL_PWM (0), e não para o pino!
    ledcWrite(CANAL_PWM, duty); 
}
