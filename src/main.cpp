#include <Arduino.h>
#include "Motor.hpp" 
#include "Display.hpp"
#include "Velocidade.hpp"
#include "StartStop.hpp"
#include "BSFC.hpp"
#include "Sensores_motor.hpp"
#include "Gerenciador_wifi.hpp" 
#include "Gerenciador_SD.hpp"
#include "Gerenciador_MQTT.hpp"

// ==========================================
// 1. INSTANCIAÇÃO GLOBAL DOS OBJETOS
// ==========================================
Motor motor;
Sensores_motor sensores;
Display display;
Gerencia_wifi wifi;
Gerencia_SD sd;
Gerenciador_MQTT mqtt;

// ==========================================
// 2. VARIÁVEIS DE CONTROLE DO SISTEMA
// ==========================================
StartStop::StatesStartStop FSMstate = StartStop::stateSwitchOFF;
unsigned long timerTelemetria = 0; 

void setup() {
    Serial.begin(115200);

    // ==========================================
    // INICIALIZAÇÃO DE HARDWARE E SENSORES
    // ==========================================
    motor.Parametros_setup_controle_motor();   
    sensores.Inicializar_setup_sensores_motor();
    
    StartStop::Inicializar_sensores_startstop(); 
    Velocidade::Inicializar_setup_sensores_velocidade(); 
    BSFC::Start_servo();

    // ==========================================
    // INICIALIZAÇÃO DE PERIFÉRICOS
    // ==========================================
    display.iniciaDisplay();
    wifi.conectar_WiFi();
    mqtt.conectar_mqtt();

    sd.AtivarSD("Tempo_ms,RPM,Velocidade,Aceleracao,Tensao,Estado_FSM");
    
}

void loop() {


    static unsigned long timerSensores = 0;
    
    // ==========================================
    // 1. ATUALIZAÇÃO DOS SENSORES
    // ==========================================

    sensores.analisaRPM();

    if (millis() - timerSensores >= 20) {
        timerSensores = millis();

        sensores.analisa_sensores_motor();
    }

    mqtt.Gerenciar_MQTT(); // Mantém reconexão e recebe JSON de parâmetros

    display.atualizaDisplay(Velocidade::calculaVelocidade(), FSMstate, sensores);

    // ==========================================
    // 2. TELEMETRIA (SD & MQTT)
    // ==========================================
    if (millis() - timerTelemetria >= 500) {
        timerTelemetria = millis();
        // --- DEBUG SERIAL ---
        Serial.print("RPM: "); Serial.print(sensores.getRpm());
        Serial.print(" | TPS: "); Serial.print(sensores.getPosBorbo());
        Serial.print(" | MAP: "); Serial.print(sensores.getMap());
        Serial.print(" | Lambda: "); Serial.print(sensores.analisaLambda());
        Serial.print(" | Bat: "); Serial.println(sensores.analisaTensao());
        
        // Enpacotamento dos dados
        float dados_envio[] = {
            sensores.getRpm(), 
            Velocidade::getVelocidade(), 
            Velocidade::getAcelera(), 
            sensores.analisaTensao(),
            (float)FSMstate, // Convertendo o estado para float só para logar
        

            
        };
        const char* nomes_dados[] = {"rpm", "vel", "acel", "bat", "fsm"};
        
        // Salva no SD (Lote de 20 linhas gerenciado pela classe)
        sd.salvarTelemetriaNoSD(dados_envio, 5);
        
        // Manda pro MQTT
        mqtt.publicar_telemetria(dados_envio, nomes_dados, 5, "ricardofonsecaj123@gmail.com/telemetria");
    }

    // ==========================================
    // 3. MÁQUINA DE ESTADOS PRINCIPAL
    // ==========================================
    switch (FSMstate) {
        case StartStop::stateSwitchOFF:
            FSMstate = StartStop::switchOFF();
            break;
            
        case StartStop::stateSwitchON:
            FSMstate = StartStop::switchON();
            break;
            
        case StartStop::stateLigaMotor:
            FSMstate = StartStop::ligaMotorSS(motor, display, sensores);
            break;
            
        case StartStop::stateDesligaMotor:
            FSMstate = StartStop::desligaMotorSS(motor, display, sensores);
            break;
            
        case StartStop::stateEstabilizaAcelera:
            FSMstate = StartStop::estabilizaAcelera(motor, sensores);
            break;
            
        case StartStop::stateStart:
            FSMstate = StartStop::start(motor, sensores);
            break;
            
        case StartStop::stateStop:
            FSMstate = StartStop::stop(motor, sensores);
            break;
            
        case StartStop::stateFreando:
            FSMstate = StartStop::freando();
            break;
            
        case StartStop::stateDesligaStartStop:
            FSMstate = StartStop::desligaStartStop(motor, display, sensores);
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