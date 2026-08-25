#include <Arduino.h>
#include "Motor.hpp" 
#include "Ckp.hpp"
#include "Lambda.hpp"
#include "Map.hpp"
#include "TPS.hpp"
#include "TENSAO.hpp" 
#include "Display.hpp"
#include "Velocidade.hpp"
#include "StartStop.hpp"
#include "BSFC.hpp"
#include "Servo.hpp"
#include "wifi.hpp" 
#include "Modulo_SD.hpp"
#include "MQTT.hpp"
#include "Callback.hpp"



#include "IMU.hpp"
#include "GPS.hpp"
#include "Hall.hpp"
#include "Filtro_Kalman_Extendido.hpp"

Motor motor;
Display display;
Gerencia_wifi wifi;
Gerencia_SD sd;
Gerenciador_MQTT mqtt;

FiltroKalmanExtendido ekf;


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
    Velocidade::Inicializar_setup_sensores_velocidade();
    
    Serial.println(">>> 5. VELOCIDADE INICIALIZADA.");
    ServoMotor::Start_servo(); 
    
    Serial.println(">>> 6. SERVO INICIALIZADO.");
    display.iniciaDisplay();
    
    // Se o log travar aqui, é o LCD I2C travando o barramento
    Serial.println(">>> 7. DISPLAY INICIALIZADO.");
    wifi.conectar_WiFi();
    
    Serial.println(">>> 8. WIFI CONECTADO.");
    mqtt.conectar_mqtt();
    
    Serial.println(">>> 9. MQTT CONECTADO.");
    mqtt.iniciarTaskMQTT();
    
    Serial.println(">>> 10. INICIALIZANDO SENSORES DE PISTA E EKF...");
    IMU::begin();
    IMU::calibrarGiroscopio();
    GPS::begin();
    ekf.init();

   sd.AtivarSD("rpm,vel,acel,map,tps,lambda,servo_atual,fsm,ekf_x,ekf_y,ekf_v,lat,lon");

    Serial.println(">>> SETUP COMPLETO COM SUCESSO! <<<");
}

void loop() {

    Velocidade::calculaVelocidade();

    static unsigned long timerSensores = 0;
    static unsigned long timerDisplay = 0;
    static unsigned long timerFSM = 0;
    static unsigned long timerEKF = 0;

    

    Ckp::analisaRPM();
    IMU::update();
    GPS::update();
    Hall::update(); 



      if (millis() - timerEKF >= 20) {
        timerEKF = millis();

        float vel_mps = Hall::getVelocidade() / 3.6f;
        
        ekf.atualizarIMU(IMU::getGyroZ(), IMU::getAccelLongitudinal(), vel_mps);

        if (GPS::getLatitude() != 0.0f && GPS::getLongitude() != 0.0f) {
            ekf.atualizarGPS(GPS::getLatitude(), GPS::getLongitude());
        }

      }


    if (millis() - timerSensores >= 20) {
        timerSensores = millis();
        Motor::analisa_sensores_motor(); 
    }


     if (millis() - timerDisplay >= 513) {
       display.atualizaDisplay(Velocidade::calculaVelocidade(), FSMstate, Tensao::getTensao());
    }

    // ==========================================
    // 2. TELEMETRIA (SD & MQTT)
    // ==========================================
    if (millis() - timerTelemetria >= 500) {
        timerTelemetria = millis();
        
        // --- DEBUG SERIAL ---
        Serial.print("RPM: "); Serial.print(Ckp::getRpm());
        Serial.print(" | TPS: "); Serial.print(TPS::getPosBorbo());
        Serial.print(" | MAP: "); Serial.print(Map::getMap());
        Serial.print(" | Lambda: "); Serial.print(Lambda::analisaLambda());
        Serial.print(" | Tensão ckp: "); Serial.println(Tensao::analisaTensao());
        Serial.print(" | RPM: "); Serial.println(Velocidade::getRPM());


          FiltroKalmanExtendido::Estado estadoEKF = ekf.getEstado();

            bool gpsValido = (GPS::getLatitude() != 0.0f && GPS::getLongitude() != 0.0f);
        
        // Enpacotamento dos dados, Os primeiros são gravado no sd
        float dados_envio[] = {
            Ckp::getRpm(), 
            Velocidade::getVelocidade(), 
            Velocidade::getAcelera(),
            Map::getMap(),
            TPS::getPosBorbo(),
            Lambda::getLambda(),
            ServoMotor::getPulsoAtual(),
            (float)FSMstate,
            estadoEKF.X,                            // 9. EKF X
            estadoEKF.Y,                            // 10. EKF Y
            estadoEKF.v,                            // 11. EKF v
            GPS::getLatitude(),                     // 12. lat
            GPS::getLongitude(),                    // 13. lon
            estadoEKF.theta,
            estadoEKF.theta * 57.2957f,
            estadoEKF.omega_bias,
            estadoEKF.ax


        };
        const char* nomes_dados[] = {"rpm", "vel", "acel", "map", "tps", "lambda","servo_atual","fsm","ekf_x", "ekf_y", "ekf_v", "lat", "lon","ekf_theta", "ekf_theta_deg", "ekf_omega_bias", "ekf_ax"};
        
        // Salva no SD (Lote de 20 linhas gerenciado pela classe)
        sd.salvarTelemetriaNoSD(dados_envio, 17);

          JsonDocument doc; 

        // 1. Variáveis do Motor e StartStop
        doc["rpm"] = Ckp::getRpm();
        doc["vel"] = Velocidade::getVelocidade();
        doc["acel"] = Velocidade::getAcelera();
        doc["map"] = Map::getMap();
        doc["tps"] = TPS::getPosBorbo();
        doc["lambda"] = Lambda::getLambda();
        doc["tensao"] = Tensao::getTensao();
        doc["servo_atual"] = ServoMotor::getPulsoAtual();
        doc["fsm"] = (int)FSMstate;

        // 2. Variáveis do EKF
        doc["ekf_x"] = estadoEKF.X;
        doc["ekf_y"] = estadoEKF.Y;
        doc["ekf_v"] = estadoEKF.v;
        doc["ekf_theta"] = estadoEKF.theta;
        doc["ekf_theta_deg"] = estadoEKF.theta * 57.2957f;
        doc["ekf_omega_bias"] = estadoEKF.omega_bias;
        doc["ekf_ax"] = estadoEKF.ax;

        // 3. Variáveis de Posição / Mapeamento
        doc["gps_valid"] = gpsValido;
        doc["lat"] = gpsValido ? GPS::getLatitude() : 0.0;
        doc["lon"] = gpsValido ? GPS::getLongitude() : 0.0;

        // Envia TODOS os dados unificados na mesma função MQTT (Tópico Telemetria Geral)
        mqtt.publicar_telemetria(doc, mqtt.topico_telemetria);
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
