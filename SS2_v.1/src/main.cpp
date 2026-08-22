#include <Arduino.h>
#include "Motor.hpp" 
#include "Ckp.hpp"
#include "Lambda.hpp"
#include "Map.hpp"
#include "TPS.hpp"
#include "TENSAO.hpp" 
#include "Display.hpp"
#include "StartStop.hpp"
#include "BSFC.hpp"
#include "Servo.hpp"
#include "wifi.hpp" 
#include "Modulo_SD.hpp"
#include "MQTT.hpp"
#include "Callback.hpp"

// Sensores de Pista, Navegação e Filtros
#include "IMU.hpp"
#include "GPS.hpp"
#include "Hall.hpp"
#include "Filtro_Kalman_Extendido.hpp"
#include "Mapeamento.hpp"

Motor motor;
Display display;
Gerencia_wifi wifi;
Gerencia_SD sd;
Gerenciador_MQTT mqtt;

// Instâncias para Estimação de Estado e Pista
FiltroKalmanExtendido ekf;
Mapeamento mapeamento;

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
    Hall::Inicializa_Hall();
    
    Serial.println(">>> 5. SENSOR HALL INICIALIZADO.");
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

    // Inicialização do SD com cabeçalho completo
    sd.AtivarSD("Timestamp,RPM,Vel,Acel,MAP,TPS,Lambda,Servo,FSM,EKF_X,EKF_Y,EKF_V,Lat,Lon,ErroLat");
    
    Serial.println(">>> SETUP COMPLETO COM SUCESSO! <<<");
}

void loop() {

    static unsigned long timerSensores = 0;
    static unsigned long timerDisplay = 0;
    static unsigned long timerFSM = 0;
    static unsigned long timerEKF = 0;

    // Leitura contínua em tempo real
    Ckp::analisaRPM(); 
    IMU::update();
    GPS::update();
    Hall::update();

    // ==========================================
    // 0. FUSÃO SENSORIAL E MAPEAMENTO DE PISTA (50Hz - 20ms)
    // ==========================================
    if (millis() - timerEKF >= 20) {
        timerEKF = millis();

        float vel_mps = Hall::getVelocidade() / 3.6f;
        
        ekf.atualizarIMU(IMU::getGyroZ(), IMU::getAccelLongitudinal(), vel_mps);

        if (GPS::getLatitude() != 0.0f && GPS::getLongitude() != 0.0f) {
            ekf.atualizarGPS(GPS::getLatitude(), GPS::getLongitude());
        }

        mapeamento.atualizarComEKF(ekf);
    }

    // ==========================================
    // 1. SENSORES DO MOTOR (20ms)
    // ==========================================
    if (millis() - timerSensores >= 20) {
        timerSensores = millis();
        Motor::analisa_sensores_motor(); 
    }

    if (millis() - timerDisplay >= 500) {
        timerDisplay = millis();
        display.atualizaDisplay(Hall::getVelocidade(), FSMstate, Tensao::getTensao());
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
        Serial.print(" | Lambda: "); Serial.print(Lambda::getLambda());
        Serial.print(" | Tensão: "); Serial.println(Tensao::getTensao());
        Serial.print(" | RPM Hall: "); Serial.println(Hall::getRPM());
        
        // Captura o estado atual do EKF e do Mapeamento
        FiltroKalmanExtendido::Estado estadoEKF = ekf.getEstado();
        Mapeamento::Dados dadosMapeamento = mapeamento.getDados();
        bool gpsValido = (GPS::getLatitude() != 0.0f && GPS::getLongitude() != 0.0f);

        // ===============================================
        // PREPARAÇÃO DOS DADOS PARA CARTÃO SD (NUMÉRICOS)
        // ===============================================
        float dados_envio[] = {
            Ckp::getRpm(),                           // 1. rpm
            Hall::getVelocidade(),                  // 2. vel
            Hall::getAceleracao(),                  // 3. acel
            Map::getMap(),                          // 4. map
            TPS::getPosBorbo(),                     // 5. tps
            Lambda::getLambda(),                    // 6. lambda
            (float)ServoMotor::getPulsoAtual(),     // 7. servo_atual
            (float)FSMstate,                        // 8. fsm
            estadoEKF.X,                            // 9. EKF X
            estadoEKF.Y,                            // 10. EKF Y
            estadoEKF.v,                            // 11. EKF v
            GPS::getLatitude(),                     // 12. lat
            GPS::getLongitude(),                    // 13. lon
            dadosMapeamento.erro_lateral_m          // 14. erro lateral
        };
        
        size_t total_dados = sizeof(dados_envio) / sizeof(dados_envio[0]);
        
        // Salva TODOS os dados numéricos no SD
        sd.salvarTelemetriaNoSD(dados_envio, total_dados);
        
        // ===============================================
        // PREPARAÇÃO DOS DADOS PARA MQTT (JSON UNIFICADO)
        // ===============================================
        JsonDocument doc; 

        // 1. Variáveis do Motor e StartStop
        doc["rpm"] = Ckp::getRpm();
        doc["vel"] = Hall::getVelocidade();
        doc["acel"] = Hall::getAceleracao();
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
        doc["pos_valida"] = dadosMapeamento.valido;
        
        // Operador ternário para evitar ponteiros nulos (Strings no JSON)
        doc["seg_atual"] = dadosMapeamento.segmento_atual_nome ? dadosMapeamento.segmento_atual_nome : "";
        doc["tipo_seg_atual"] = dadosMapeamento.tipo_segmento_atual ? dadosMapeamento.tipo_segmento_atual : "";
        doc["prox_seg"] = dadosMapeamento.proximo_segmento_nome ? dadosMapeamento.proximo_segmento_nome : "";
        doc["tipo_prox_seg"] = dadosMapeamento.tipo_proximo_segmento ? dadosMapeamento.tipo_proximo_segmento : "";
        
        doc["dist_prox_seg"] = dadosMapeamento.distancia_proximo_segmento_m;
        doc["dist_acum"] = dadosMapeamento.dist_acum_m;
        doc["erro_lat"] = dadosMapeamento.erro_lateral_m;

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
                FSMstate = StartStop::estabilizaAcelera(motor);
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