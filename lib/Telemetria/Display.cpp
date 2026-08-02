#include "Display.hpp"
#include "Velocidade.hpp"
#include "StartStop.hpp"

// NOVA FUNÇÃO: Arruma o display automaticamente se o fio soltar e voltar
void Display::autoReparo() {
    Wire.beginTransmission(0x27);
    bool temConexao = (Wire.endTransmission() == 0); // Pinga o display

    if (temConexao && !displayConectado) {
        // O fio reconectou! Inicializa de novo para tirar os "caracteres doidos"
        this->lcd.init();
        this->lcd.backlight();
        this->lcd.clear();
        displayConectado = true;
    } else if (!temConexao) {
        // Fio soltou, avisa o sistema para não tentar escrever
        displayConectado = false;
    }
}

void Display::iniciaDisplay () {
  Wire.begin();
  
  // Timeout para evitar que o Arduino trave inteiro se o I2C desconectar
  #if defined(ARDUINO_ARCH_AVR) || defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true); 
  #endif

  Wire.setClock(100000); 

  // Chama o reparo a primeira vez para testar se ligou conectado
  this->autoReparo();

  if(displayConectado) {
    this->lcd.setCursor(0,0);
    this->lcd.print("Iniciando Ben 10");
    delay(500);
    this->lcd.clear();
  }
}

// Removida a checagem dupla do millis aqui (já é feita no atualiza)
void Display::mostraTensaoEVel(float velocidade, float tensao){
    this->lcd.setCursor(0,0);
    this->lcd.print("ace:    ");
    this->lcd.setCursor(5,0);
    this->lcd.print(Velocidade::getAcelera());

    this->lcd.setCursor(0,1);
    this->lcd.print("Vel:            ");
    this->lcd.setCursor(6,1);
    this->lcd.print(velocidade);

    this->lcd.setCursor(12,1);
    this->lcd.print(tensao); 
}

void Display::atualizaDisplay( float velocidade, int FSMState, float tensao) {

  // O relógio é acionado a cada 200ms
  if ((millis() - timeOld) >= timeInterval) {

    // 1. RODA O REPARO SILENCIOSO
    this->autoReparo();

    // 2. SÓ ATUALIZA A TELA SE O FIO ESTIVER CONECTADO
    if (displayConectado) {
      
      mostraTensaoEVel(velocidade, tensao);

      String FSMState_str;
      switch (FSMState)
      {
        case StartStop::stateSwitchOFF: FSMState_str = "SS_off"; break;
        case StartStop::stateSwitchON: FSMState_str = "SS__on"; break;
        case StartStop::stateDesligaMotor: FSMState_str = "deslgM"; break;
        case StartStop::stateLigaMotor: FSMState_str = "ligaM "; break;
        case StartStop::stateEstabilizaAcelera: FSMState_str = "estabA"; break;
        case StartStop::stateStart: FSMState_str = "iniciou"; break;
        case StartStop::stateStop: FSMState_str = "parou"; break;
        case StartStop::stateFreando: FSMState_str = "freou"; break;
        case StartStop::stateNotLigou: FSMState_str = "notLig"; break;
        case StartStop::stateNotDesligou: FSMState_str = "notDsg"; break;
        case StartStop::stateDesligaStartStop: FSMState_str = "deslgSS"; break;
        default: break;
      }
      
      this->lcd.setCursor(10, 0);
      this->lcd.print(FSMState_str);
    }
    
    // Atualiza o tempo no final (independente de estar com mau contato ou não)
    timeOld = millis();
  }
}