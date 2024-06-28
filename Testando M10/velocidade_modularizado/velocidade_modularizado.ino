#include "Arduino.h"

#define pinSensorHall      DD2

#define velZERO            150  // Valores para teste
#define velMin             400  // Valores para teste
#define velMax             700  // Valores para teste
#define taxaAtualizacaoVel 1000 // ms
#define circunfRoda        0.47 // m
#define pulsosPorVolta     5
#define sampleSize         3    // Numero de amostras para calcular velocidade

#define MPS_to_KMPH 3.6  // metros por segundo p/ quilometro por hora
#define MS_to_S     1000 // milisegundos p/ segundos

unsigned long lastPulseTime; // ms
unsigned long currentMillis; // ms
unsigned long timeContador;  // ms
unsigned long pulseTime;     // ms
float speed;                 // km/h
float lastSpeed;             // km/h         

unsigned long pulseTimes[sampleSize];
int pulseIndex;

/** 
 * @brief Armazena o valor de periodo da onda no vetor usado para o calculo de velocidade
 * 
 */
void calc();

/**
 * @brief Atualiza a velocidade com base nos pulsos do Sensor Hall 
 * 
 * @details DEFINIR
 * 
 * @param velocidadeAtual velocidade antes da chamada da funcao.
 * 
 * @return velocidade (m/s)
 */
void calculaVelocidade(float speed);

void setup() {
  Serial.begin(115200);

  pinMode(pinSensorHall, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);

  lastPulseTime = 0;
  currentMillis = 0;
  timeContador = 0;
  pulseTime = 0;
  speed = 0.0;
  lastSpeed = 0;
  for (int i = 0; i < sampleSize; i++) { pulseTimes[i] = 0; }
  pulseIndex = 0;
}

void loop() {
    calculaVelocidade(speed);

    // Serial.print("VelocidadeAtual:");
    // Serial.println(velocidadeAtual);
}

void calc() {
  currentMillis = millis();
  pulseTime = currentMillis - lastPulseTime;
  timeContador = millis();
  lastPulseTime = currentMillis;
  pulseTimes[pulseIndex] = pulseTime;
  pulseIndex = (pulseIndex + 1) % sampleSize; 
  lastPulseTime = pulseTime; 
}

void calculaVelocidade(float speed) {
  currentMillis = millis();
  pulseTime = currentMillis - lastPulseTime;

  if(pulseTime >= taxaAtualizacaoVel) {
    currentMillis = millis();
    lastPulseTime = currentMillis;

    detachInterrupt(digitalPinToInterrupt(pinSensorHall));
    unsigned long averagePulseTime = 0;  // ms
    for (int i = 0; i < sampleSize; i++) {
      averagePulseTime += pulseTimes[i];
    }
    averagePulseTime /= sampleSize;
    if(speed >= 1.9*lastSpeed && speed != 0){
    speed = lastSpeed;
    } else if(speed <= 0.4*lastSpeed && speed != 0){
    speed = lastSpeed;
    } else {
    speed = circunfRoda/(pulsosPorVolta*averagePulseTime) * MS_to_S * MPS_to_KMPH;
    }
    lastSpeed = speed;
    attachInterrupt(digitalPinToInterrupt(pinSensorHall), calc, RISING);
    Serial.print("Speed:");
    Serial.println(speed);
  }
}
