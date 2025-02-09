#include <Arduino.h>
#include <arduinoFFT.h>
#include "PressureSensor.h" // Pressure sensor reading library
#include <Wire.h>

#define SAMPLES 128 // Must be a power of 2 (e.g., 64, 128, etc.)
#define SAMPLING_FREQUENCY 1850.0f // Sampling frequency in Hz

float vReal[SAMPLES];  // Real part of FFT input
float vImag[SAMPLES];  // Imaginary part (initialize to 0)
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  unsigned long startTime = micros();  // Record start time
  unsigned long nextSampleTime = startTime;

  // Read data from the sensor
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = getPressure();
    vImag[i] = 0;  // Initialize imaginary part to 0
    nextSampleTime += (1000000 / SAMPLING_FREQUENCY); // Wait for next sample
    while (micros() < nextSampleTime) {
      // Wait until it's time for the next sample
    }
  }

  unsigned long elapsedTime = micros() - startTime;  // Calculate elapsed time in microseconds
  float actualSamplingFrequency = (1000000.0f * SAMPLES) / elapsedTime;  // Compute effective sampling frequency
  Serial.print("Actual Sampling Frequency: ");
  Serial.println(actualSamplingFrequency);

  // Perform FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Send FFT data for visualization
  Serial.print("<FFT>"); // Start marker
  for (int i = 0; i < SAMPLES / 2; i++) { // Only send first half of FFT (positive frequencies)
    Serial.print(vReal[i]);
    if (i < SAMPLES / 2 - 1) {
      Serial.print(","); // Separate values with commas
    }
  }
  Serial.println("</FFT>"); // End marker
  Serial.println();

  // Find the most prevalent frequency
  double peakFrequency = FFT.majorPeak();

  // Output result
  Serial.print("<PEAK>");
  Serial.print(peakFrequency);
  Serial.println("</PEAK>");
  Serial.println();
}