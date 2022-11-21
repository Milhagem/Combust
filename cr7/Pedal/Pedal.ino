#define POT_ACELERADOR A8

#define pedalGND A8
#define pedal A9
#define pedalVCC A10

void setup() {
  Serial.begin(9600);
  pinMode(pedal,INPUT);
}

void loop() {
  Serial.print("Acelerador: ");
  Serial.println(analogRead(pedal));
  delay(100);
}
