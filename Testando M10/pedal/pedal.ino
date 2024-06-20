#define pedalGND A8
#define pedal A9
#define pedalVcc A10

void setup() {
  Serial.begin(9600);
  pinMode(pedalGND, OUTPUT);
  pinMode(pedal, INPUT);
  pinMode(pedalVcc, OUTPUT);
  analogWrite(pedalGND, 0); // Pino 8 -> GND 
  analogWrite(pedalVcc, 1023); // Pino 10 -> VCC
}

void loop() {
  Serial.print("Acelerador: ");
  Serial.println(analogRead(pedal));
  delay(100);
}
