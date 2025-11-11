#include <Arduino.h>
#include <arduinoFFT.h>
#include "PressureSensor_rpPico.h"
#include <Wire.h>

#define SAMPLES 256                 // Must be a power of 2
#define SAMPLING_FREQUENCY 3000.0f  // Sampling frequency

// FFT arrays
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Calculate sampling period in microseconds
const unsigned long SAMPLING_PERIOD_US = round(1000000.0f / SAMPLING_FREQUENCY);

// Buffer for storing timestamp of each sample
unsigned long sampleTimestamps[SAMPLES];

void setup() {
  initialize(14, 15);
  Serial.begin(115200);
  
  while (!Serial) {
    delay(10);
  }
}

void printTimingStats() {
  unsigned long totalInterval = 0;
  unsigned long minInterval = UINT32_MAX;
  unsigned long maxInterval = 0;
  
  for (int i = 1; i < SAMPLES; i++) {
    unsigned long interval = sampleTimestamps[i] - sampleTimestamps[i-1];
    totalInterval += interval;
    minInterval = min(minInterval, interval);
    maxInterval = max(maxInterval, interval);
  }
  
  float avgInterval = float(totalInterval) / (SAMPLES - 1);
  float actualFreq = 1000000.0f / avgInterval;
  
  Serial.print("Min interval (us): "); Serial.println(minInterval);
  Serial.print("Max interval (us): "); Serial.println(maxInterval);
  Serial.print("Avg interval (us): "); Serial.println(avgInterval);
  Serial.print("Actual frequency (Hz): "); Serial.println(actualFreq);
  Serial.print("Jitter (us): "); Serial.println(maxInterval - minInterval);
}

void loop() {
  noInterrupts();
  
  unsigned long startTime = micros();
  unsigned long nextSampleTime = startTime;

  // Populate FFT array
  for (uint16_t i = 0; i < SAMPLES; i++) {
    // Wait until next sample time
    while (micros() < nextSampleTime) {}
    
    // Record actual timestamp
    sampleTimestamps[i] = micros();
    
    // Read sensor and store value
    vReal[i] = getPressure();
    vImag[i] = 0;
    
    // Calculate next sample time
    nextSampleTime = startTime + (i + 1) * SAMPLING_PERIOD_US;
  }
  
  interrupts();

  // Print timing statistics
  printTimingStats();

  // Perform FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Write FFT data to serial for the visualizer
  Serial.print("<FFT>");
  for (int i = 0; i < SAMPLES / 2; i++) {
    Serial.print(vReal[i], 4);
    if (i < SAMPLES / 2 - 1) {
      Serial.print(",");
    }
  }
  Serial.println("</FFT>");

  // Find and output peak frequency
  const uint16_t peakFrequency = round(FFT.majorPeak());
  Serial.print("<PEAK>");
  Serial.print(peakFrequency);
  Serial.println("</PEAK>");

  int peakIndex = (int)((peakFrequency * SAMPLES) / SAMPLING_FREQUENCY);
  double peakAmplitude = vReal[peakIndex];

  Serial.print("Amplitude: ");
  Serial.println(vReal[peakIndex], 4);

  delay(100);
}