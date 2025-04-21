#include "PressureSensor_rpPico.h"
#include <vector>
#include <type_traits>

std::vector<float> pressures;

// Returns (sd, avg)
std::pair<double, double> computeSdAvg(const std::vector<float> data) {
  double sum = 0.0;
  for (int i = 0; i < data.size(); i++) {
    sum += data[i];
  }

  double avg = sum / data.size();

  double sdSum = 0.0;
  for (int i = 0; i < data.size(); i++) {
    sdSum += pow(data[i] - avg, 2);
  }

  double sd = sqrt(sdSum / (data.size() - 1));
  return std::make_pair(sd, avg);
}

void setup() {
  Serial.begin(115200);
  initialize(14, 15);

  pinMode(7, OUTPUT); // LED for visualization

  while (pressures.size() < 50) {
    pressures.push_back(getPressure());
  }
}

void loop() {
  float pressure = getPressure();

  std::pair<double, double> sdAvg = computeSdAvg(pressures);
  double zScore = (pressure - sdAvg.second) / sdAvg.first;

  if (zScore > 6.0) {
    Serial.println("Tongue");
    digitalWrite(7, HIGH);
    delay(100);
  }
  else {
    digitalWrite(7, LOW);
  }

  // Sliding window
  pressures.erase(pressures.begin());
  pressures.push_back(pressure);
}