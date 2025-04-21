#include "PressureSensor_rpPico.h"

float prevPres = 0;

void setup() {
  Serial.begin(115200);
  initialize(14, 15);
  float pressure = getPressure();
  prevPres = pressure;
}

void loop() {
  float pressure = getPressure();
  Serial.println(pressure);
}