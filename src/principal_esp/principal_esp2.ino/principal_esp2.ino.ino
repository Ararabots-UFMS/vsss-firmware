void giro(void * pvParameters){
  Serial.println("\t \t giro");
}

void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(giro, "giro", 10000, NULL, 0, NULL, 1);
}

void loop() {
  Serial.println("loop");
  delay(20);
}
