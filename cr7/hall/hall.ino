
int hallPin = 19;
int ledRed = 2;

void setup() {
  pinMode(hallPin, INPUT);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(ledRed, OUTPUT);
  digitalWrite(0, LOW); // Desliga LED verde
  digitalWrite(1, LOW); // Desliga LED azul
  digitalWrite(ledRed, LOW);
}

void loop() {
  while(digitalRead(hallPin) == HIGH){
    digitalWrite(ledRed, LOW);
  }
  digitalWrite(ledRed, HIGH);

}
