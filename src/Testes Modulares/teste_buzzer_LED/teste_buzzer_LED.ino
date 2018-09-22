#define LED 22
#define BUZZER 19


void setup() {
    pinMode(LED, OUTPUT);
    pinMode(BUZZER, OUTPUT);
}

void loop() {

  digitalWrite(LED,HIGH);
  digitalWrite(BUZZER.HIGH);
  delay(2000);
  digitalWrite(LED,LOW);
  digitalWrite(BUZZER,LOW);
  delay(1000);

}
