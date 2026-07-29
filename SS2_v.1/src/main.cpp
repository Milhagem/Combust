#include "BSFC.hpp"
#include "Callback.hpp"
#include "CKP.hpp"
#include "Display.hpp"
#include "Gerenciador_WiFi.hpp" 
#include "Hall.hpp"
#include "Lambda.hpp"
#include "LM2907.hpp" 
#include "MAP.hpp"
#include "Modulo_SD.hpp"
#include "Motor.hpp"
#include "MQTT.hpp"
#include "Servo.hpp"
#include "StartStop.hpp"
#include "TPS.hpp"

#include <Arduino.h>



Display display;
Gerenciador_WiFi wifi;
Hall hall;
Modulo_SD sd;
Motor motor;
MQTT mqtt;



StartStop::StatesStartStop FSMstate = StartStop::stateSwitchOFF;
unsigned long timerTelemetria = 0; 

void setup() {
    Serial.begin(115200);
    delay(2000); // Dá tempo para abrir o monitor serial com calma
    Serial.println(">>> 1. INICIANDO SETUP...");
    
    Callback::carregarParametrosIniciais(); 
    Serial.println(">>> 2. CARREGOU MEMORIA NVS.");
    
    Motor::Parametros_setup_controle_e_sensores_motor(); 
    // Se o log travar aqui, o culpado é o pino da interrupção (ruído ou conflito de hardware)
    Serial.println(">>> 3. MOTOR/CKP INICIALIZADO.");
    
    StartStop::Inicializar_sensores_startstop(); 
    // Se o log travar aqui, o culpado está dentro do StartStop
    Serial.println(">>> 4. START-STOP INICIALIZADO.");
    
    // Velocidade::Inicializar_setup_sensores_velocidade();
    Serial.println(">>> 5. VELOCIDADE INICIALIZADA.");
    
    Servo::Start_servo(); 
    Serial.println(">>> 6. SERVO INICIALIZADO.");
    
    display.iniciaDisplay();
    // Se o log travar aqui, é o LCD I2C travando o barramento
    Serial.println(">>> 7. DISPLAY INICIALIZADO.");
    
    wifi.conectar_WiFi();
    Serial.println(">>> 8. WIFI CONECTADO.");
    
    mqtt.conectar_mqtt();
    Serial.println(">>> 9. MQTT CONECTADO.");
    mqtt.iniciarTaskMQTT();
    
    Serial.println(">>> SETUP COMPLETO COM SUCESSO! <<<");
}

void loop() {

    static unsigned long timerSensores = 0;
    static unsigned long timerDisplay = 0;
    static unsigned long timerFSM = 0;

    

    CKP::analisaRPM(); 

    if (millis() - timerSensores >= 20) {
        timerSensores = millis();
        Motor::analisa_sensores_motor(); 
    }

     if (millis() - timerDisplay >= 500) {
      hall.update();
      display.atualizaDisplay(hall.getVelocidade(), FSMstate, LM2907::getTensao());
    }

    // ==========================================
    // 2. TELEMETRIA (SD & MQTT)
    // ==========================================
    if (millis() - timerTelemetria >= 500) {
        timerTelemetria = millis();
        
        // --- DEBUG SERIAL ---
        Serial.print("RPM: "); Serial.print(CKP::getRpm());
        Serial.print(" | TPS: "); Serial.print(TPS::getPosBorbo());
        Serial.print(" | MAP: "); Serial.print(MAP::getMap());
        Serial.print(" | Lambda: "); Serial.print(Lambda::analisaLambda());
        Serial.print(" | Tensão ckp: "); Serial.println(LM2907::analisaTensao());
        Serial.print(" | RPM: "); Serial.println(hall.getRPM());
        
        // Enpacotamento dos dados, Os primeiros são gravado no sd
        float dados_envio[] = {
            CKP::getRpm(), 
            hall.getVelocidade(), 
            hall.getAceleracao(),
            MAP::getMap(),
            TPS::getPosBorbo(),
            Lambda::getLambda(),
            Servo::getPulsoAtual(),
            (float)FSMstate 
        };
        const char* nomes_dados[] = {"rpm", "vel", "acel", "map", "tps", "lambda","servo_atual","fsm"};
        
        // Salva no SD (Lote de 20 linhas gerenciado pela classe)
        sd.salvarTelemetriaNoSD(dados_envio, 6);
        
        // Manda pro MQTT
        mqtt.publicar_telemetria(dados_envio, nomes_dados, 8, "ricardofonsecaj123@gmail.com/telemetria");
    }

    // ==========================================
    // 3. MÁQUINA DE ESTADOS PRINCIPAL a 100hz
    // ==========================================
    
    if (millis() - timerFSM >= 10) { // Garante o dt fixo de 10ms
        timerFSM = millis();
    switch (FSMstate) {
        case StartStop::stateSwitchOFF:
            FSMstate = StartStop::switchOFF();
            break;
            
        case StartStop::stateSwitchON:
            FSMstate = StartStop::switchON();
            break;
            
        case StartStop::stateLigaMotor:
            FSMstate = StartStop::ligaMotorSS(motor, display);
            break;
            
        case StartStop::stateDesligaMotor:
            FSMstate = StartStop::desligaMotorSS(motor, display);
            break;
            
        case StartStop::stateEstabilizaAcelera:
            FSMstate = StartStop::estabilizaAcelera(motor); // CORRIGIDO: Removidos parâmetros excedentes
            break;
            
        case StartStop::stateStart:
            FSMstate = StartStop::start(motor);
            break;
            
        case StartStop::stateStop:
            FSMstate = StartStop::stop(motor);
            break;
            
        case StartStop::stateFreando:
            FSMstate = StartStop::freando();
            break;
            
        case StartStop::stateDesligaStartStop:
            FSMstate = StartStop::desligaStartStop(motor, display);
            break;
            
        case StartStop::stateNotLigou:
            FSMstate = StartStop::notLigou(display);
            break;
            
        case StartStop::stateNotDesligou:    
            FSMstate = StartStop::notDesligou(display);
            break;
            
        default:
            FSMstate = StartStop::stateDesligaStartStop;
            break;
    }
      
}
}