#include <Servo.h>

#define ligaMotor     13  //Saída liga motor
#define desligaMotor  12  //Saída desliga motor
#define freio         11  //Entrada sensor freio
#define SERVOPIN      8   // Saída Digital 8 PWM
#define switchSS      10  //Entrada chave liga desliga start/stop (Z)

#define FALSE 0
#define TRUE 1
#define LIGADO  1
#define DESLIGADO 0
#define vecMin  100
#define vecMax  300
#define ZEROvec  50
#define tensaoLigado 5 //valor da tensao que indica que o motor esta ligado (será definido ainda)


Servo servo; // Variável Servo
int pos=0; // Posição Servo
//boolean on_off; //Chave liga desliga autopilot
boolean estadoMotor = DESLIGADO;
//int vec;  //Valor de velocidade(será definido ainda)
int FSMstate = 0;

int valorInicial; 
float tensao; //valor da tensao lida pelo arduino

void setup() {
  servo.attach(SERVOPIN);
  
  pinMode(ligaMotor,OUTPUT);
  pinMode(desligaMotor,OUTPUT);
  pinMode(freio,INPUT_PULLUP);  //(Fa)
  pinMode(switchSS,INPUT_PULLUP); //(Z)
  pinMode(A0,INPUT);  //velocidade atual
  pinMode(A1,INPUT);  //comparador de tensao 
  pinMode(9,OUTPUT);
  
  servo.write(0); // Inicia motor posição zero
  estadoMotor = DESLIGADO;
  digitalWrite(ligaMotor,LOW);
  digitalWrite(desligaMotor,LOW);
  
  Serial.begin(9600);
}
//******************40 == velocidade 0 -> está com esse valor para fins de simulação
void loop() {
  switch (FSMstate)
  {//////////////////////////////////////////////////////////////////////////////////////////////////////////////
     case 0: // ON/OFF  Estado em que o start/stop está desligado
       if(digitalRead(switchSS) == 0 && analogRead(A0)>vecMin){//liga start/stop
        FSMstate = 1;
       }
       Serial.println("FSMstate:0"); 
     break;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
     case 1://Monitoramento Velocidade
       //Caso a velocidade esteja abaixo de 29km/h e o motor esteja desligado
       if(digitalRead(switchSS) == 0 && analogRead(A0)<vecMax && analogRead(A0)>ZEROvec && estadoMotor == DESLIGADO && digitalRead(freio) == 1){
        ligaMotorFunction(); 
        FSMstate = 2;
       }
       //Caso a velocidade esteja abaixo de 29km/h e o motor esteja ligado
       else if(digitalRead(switchSS) == 0 && analogRead(A0)<vecMax && analogRead(A0)>ZEROvec && estadoMotor == LIGADO && digitalRead(freio) == 1){
        FSMstate = 2;
       }
       else if(digitalRead(switchSS) == 0 && analogRead(A0)>vecMax && digitalRead(freio) == 1){//Caso a velocidade esteja acima de 29 km/h
        desligaMotorFunction(); 
        FSMstate = 3;
       }
       else if(digitalRead(switchSS) == 0 && digitalRead(freio) == 0 ){// Freio pressionado
        FSMstate = 5;
       }
       else if(digitalRead(switchSS) == 1){//Ativa modo manual
        pos=0;
        servo.write(pos);
        FSMstate = 0;
       }
       Serial.println("FSMstate = 1");
     break;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
     case 2://incremento aceleracao
       if( digitalRead(switchSS) == 0 && digitalRead(freio) == 1){//Caso o freio esteja solto
          if(pos<=80) //Aumenta a abertura do borboleta
            pos+=10;
          servo.write(pos); 
          Serial.print("Velocidade: "); 
          Serial.print(analogRead(A0));
          Serial.print("    ; Posicao: "); 
          Serial.println(pos);
          Serial.println("Motor ligado");
          FSMstate = 1;
       }
       else if(digitalRead(switchSS) == 0 && digitalRead(freio) == 0 ){// Freio pressionado
        FSMstate = 5;
       }
       if(digitalRead(switchSS) == 1){//Ativa modo manual
        pos=0;
        servo.write(pos);
        FSMstate = 0;
       }
       Serial.println("FSMstate = 2");
     break;
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
     case 3://desliga motor
      if(analogRead(A0)<vecMin && digitalRead(switchSS) == 0 && digitalRead(freio) == 1){//Velocidade < 19km/h
        ligaMotorFunction();
        FSMstate = 4;
      }
      else if(digitalRead(switchSS) == 0 && digitalRead(freio) == 0 ){// Freio pressionado
        FSMstate = 5;
      }
      if(digitalRead(switchSS) == 1){//Ativa modo manual
        pos=0;
        servo.write(pos);
        FSMstate = 0;
      }
      Serial.println("FSMstate = 3"); 
     break;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     case 4://liga motor
      if(  digitalRead(switchSS) == 0 && digitalRead(freio) == 1 ){//&& digitalRead(Lx)==1){//Resposta da comparacao do sinal LM2097 (diferenca de tensao se manteve 2V)
        FSMstate = 1;
      }
      else if(digitalRead(switchSS) == 0 && digitalRead(freio) == 0 ){// Freio pressionado
        FSMstate = 5;
      }
      if(digitalRead(switchSS) == 1){//Ativa modo manual
        pos=0;
        servo.write(pos);
        FSMstate = 0;
      }
      Serial.println("FSMstate = 4"); 
     break;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
case 5://freou
      if(digitalRead(freio) == 0){//Freio pressionado 
            pos=0;
            servo.write(pos);
            Serial.println("Freio");
            Serial.print("Posicao: "); 
            Serial.println(pos);
       }
      else if(digitalRead(switchSS) == 0 && digitalRead(freio) == 1 ){// Freio solto
        FSMstate = 1;
      }
      if(digitalRead(switchSS) == 1){//Ativa modo manual
        pos=0;
        servo.write(pos);
        FSMstate = 0;
      }
      Serial.println("FSMstate = 5"); 
     break;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     default:
      FSMstate = 0;
  }
}

void ligaMotorFunction(){
      digitalWrite(ligaMotor,HIGH);//
      Serial.println("Ligando Motor");
      int i = 0;
      while(i<10){ //verifica se a tensao esta dentro dos valores predeterminados (no caso 4 a 6) por 1 segundo (valores serao definido ainda)
        delay(100);
        analiseTensao();
        if(tensao<tensaoLigado+1 && tensao>tensaoLigado-1)
          i++;
        else i=0;
      }
      digitalWrite(ligaMotor,LOW);  //Liga motor de arranque por 800ms
      estadoMotor = LIGADO;
      digitalWrite(9,HIGH);
      Serial.println("Motor ligado");
      Serial.print("Velocidade: "); 
      Serial.println(analogRead(A0)); 
}

void desligaMotorFunction(){
      //Após velocidade máx ser alcançada, desliga motor e volta servo pra posição inicial
      pos=0;
      servo.write(pos);
      digitalWrite(desligaMotor,HIGH);
      digitalWrite(9,LOW);
      delay(800);
      digitalWrite(desligaMotor,LOW);
      estadoMotor = DESLIGADO;
      Serial.println("Motor desligado");
      Serial.print("Velocidade: "); 
      Serial.println(analogRead(A0)); 
}

void analiseTensao(){
  valorInicial = analogRead(A1);
  tensao = (valorInicial*5.0) / 1024; 
}
