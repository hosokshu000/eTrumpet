#include "PressureSensor_rpPico.h"

void setup() {
  Serial.begin(115200);
  initialize(14, 15);
}

void loop() {
  float pressure = getPressure();
  Serial.println(pressure);
}