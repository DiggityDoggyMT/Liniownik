void setup() {
Serial.begin(9600);
}

void loop() {
for (int pin = 2; pin < 14; pin++) { // For a typical Arduino Uno

Serial.print(pin);
Serial.print(" ");
digitalWrite(pin, HIGH); //Włączenie diody
  delay(1000); //Odczekanie 1 sekundy
  digitalWrite(pin, LOW); //Wyłączenie diody
  delay(1000); //Odczekanie jednej sekundy
}
delay(1000);
}
