
void setup() {
  // Comunicacion con el computador por USB
  Serial.begin(115200);

  // Comunicacion con HC-05 en modo AT
  Serial1.begin(38400);

  delay(1000);

  Serial.println("=== INNOVA: CONFIGURACION HC-05 ===");
  Serial.println("Escribe AT y presiona Enter");
}

void loop() {
  // Computador -> HC-05
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }

  // HC-05 -> Computador
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
