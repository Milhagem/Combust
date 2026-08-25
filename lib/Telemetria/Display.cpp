#include "Display.hpp"
#include "Velocidade.hpp"
#include "StartStop.hpp"
#include "Motor.hpp"
#include "Ckp.hpp"
#include <WiFi.h>

void Display::autoReparo() {
    Wire.beginTransmission(0x27);
    bool temConexao = (Wire.endTransmission() == 0);

    if (temConexao && !displayConectado) {
        this->lcd.init();
        this->lcd.backlight();
        this->lcd.clear();
        displayConectado = true;
    } else if (!temConexao) {
        displayConectado = false;
    }
}

void Display::iniciaDisplay () {
    Wire.begin();
    
    #if defined(ARDUINO_ARCH_AVR) || defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true); 
    #endif

    Wire.setClock(100000); 

    this->autoReparo();

    if(displayConectado) {
        this->lcd.setCursor(0,0);
        this->lcd.print("Iniciando ECU...");
        delay(500);
        this->lcd.clear();
    }
}

// Mantida por segurança caso seja chamada em outro lugar
void Display::mostraTensaoEVel(float velocidade, float tensao){
    // ... seu código antigo ou pode deixar vazio se não usar mais avulsa ...
}

void Display::atualizaDisplay(float velocidade, int FSMState, float tensao) {
  
  if ((millis() - timeOld) >= timeInterval) {
    this->autoReparo();

    if (displayConectado) {
      
      // ==========================================
      // BUSCA ATIVA DE DADOS (Sem mudar a main)
      // ==========================================
      float rpm = Ckp::getRpm();
      bool wifiOk = (WiFi.status() == WL_CONNECTED);
      bool motorLigado = (Motor::analisa_status_motor() == Motor::engineON);

      // ==========================================
      // CRONÔMETRO DE TEMPO DO MOTOR
      // ==========================================
      if (motorLigado) {
          if (!motorEstavaLigado) {
              timerMotorLigado = millis();
              motorEstavaLigado = true;
          }
          tempoTotalLigado = (millis() - timerMotorLigado) / 1000.0f;
      } else {
          motorEstavaLigado = false;
          tempoTotalLigado = 0.0f; // Zera se o motor desligar
      }

      // ==========================================
      // ABREVIAÇÃO DA FSM (Máximo 4 Caracteres)
      // ==========================================
      const char* FSMState_str = "    ";
      switch (FSMState) {
        case StartStop::stateSwitchOFF:         FSMState_str = "SOFF"; break;
        case StartStop::stateSwitchON:          FSMState_str = "S_ON"; break;
        case StartStop::stateDesligaMotor:      FSMState_str = "DESL"; break;
        case StartStop::stateLigaMotor:         FSMState_str = "LIGA"; break;
        case StartStop::stateEstabilizaAcelera: FSMState_str = "ESTA"; break;
        case StartStop::stateStart:             FSMState_str = "STRT"; break;
        case StartStop::stateStop:              FSMState_str = "STOP"; break;
        case StartStop::stateFreando:           FSMState_str = "FREO"; break;
        case StartStop::stateNotLigou:          FSMState_str = "NLIG"; break;
        case StartStop::stateNotDesligou:       FSMState_str = "NDES"; break;
        case StartStop::stateDesligaStartStop:  FSMState_str = "DSS "; break;
        default: break;
      }

      // ==========================================
      // MONTAGEM DA TELA
      // ==========================================
      char linha0[17];
      char linha1[17];
      
      char charWifi = wifiOk ? 'W' : '-';

      // Linha 0: "W T:12.34   ESTA" (16 chars)
      snprintf(linha0, sizeof(linha0), "%c T:%-6.2f %4s", charWifi, tempoTotalLigado, FSMState_str);
      
      // Linha 1: "V:15.3 R:3500   " (16 chars)
      snprintf(linha1, sizeof(linha1), "V:%-5.1f R:%-4.0f ", velocidade, rpm);

      this->lcd.setCursor(0, 0);
      this->lcd.print(linha0);
      
      this->lcd.setCursor(0, 1);
      this->lcd.print(linha1);
    }
    
    timeOld = millis();
  }
}