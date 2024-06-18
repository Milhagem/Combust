#include <Arduino.h>
#include <stdlib.h>
#include <Wire.h>

#define SENSOR_HALL DD3

volatile int pico = 0;
const int taxaAtualizacao = 1000; // Intervalo de tempo entre as medições
unsigned long oldtime;

void variador(){
  pico++;
} 

void setup(){
  Serial.begin(115220);

  pinMode(SENSOR_HALL, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_HALL), variador, FALLING);

  oldtime = 0; // ms
}

void loop(){
  if((millis() - oldtime) > taxaAtualizacao){ 
    
    detachInterrupt(digitalPinToInterrupt(DD3));
    oldtime = millis();
    pico = 0;

    attachInterrupt(digitalPinToInterrupt(SENSOR_HALL), variador, FALLING);
  }

  Serial.print("Pico:");
  Serial.println(pico);
}