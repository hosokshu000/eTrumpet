const uint8_t valvePins[] = {26, 27, 28};

void setup() {
  Serial.begin(9600);
}

void loop() {
  float val = 0;
  for (int i = 0; i < 3; i++) {
    val = analogRead(valvePins[i]);

    if (i >= 2) {
      Serial.println(val > 550);
    }
    else {
      Serial.print(val > 550);
      Serial.print(" ");
    }
  }
}