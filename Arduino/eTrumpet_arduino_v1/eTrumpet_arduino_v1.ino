#include <Arduino.h>
#include <arduinoFFT.h>
#include "PressureSensor.h"  // Pressure sensor reading library
#include <Wire.h>

#define SAMPLES 128                 // Must be a power of 2
#define SAMPLING_FREQUENCY 1850.0f  // Sampling frequency in Hz

float vReal[SAMPLES];  // Real part of FFT input
float vImag[SAMPLES];  // Imaginary part
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Pin setup
const uint8_t valvePins[] = {2, 4, 7};
const uint8_t piezo = 8;

// Array to store frequency values
const uint16_t frequencyChart[7][7] = {
  {233, 349, 466, 587, 698, 831, 932},
  {220, 330, 440, 554, 659, 784, 880},
  {208, 311, 415, 523, 622, 740, 831},
  {196, 294, 392, 494, 587, 698, 784},
  {185, 277, 370, 466, 554, 659, 740},
  {175, 262, 349, 440, 523, 622, 698},
  {165, 247, 330, 415, 494, 587, 659}
};

const float SAMPLING_PERIOD = 1000000.0f / SAMPLING_FREQUENCY;

uint16_t currentFrequency = 0;  // Track current frequency to avoid redundant tone updates

void computeHarmonicSeries(const uint8_t* valves, float* harmonicSeries);
uint16_t getClosestPitch(const float* harmonicSeries, float fftFrequency);

void setup() {
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(valvePins[i], INPUT);
  }
  pinMode(piezo, OUTPUT);
  
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);
}

void loop() {
  // FFT with pressure sensor input
  unsigned long startTime = micros();  // Record start time
  unsigned long nextSampleTime = startTime;

  // Read data from the sensor
  for (uint8_t i = 0; i < SAMPLES; i++) {
    vReal[i] = getPressure();
    vImag[i] = 0;                                      // Initialize imaginary part to 0
    nextSampleTime += SAMPLING_PERIOD;  // Wait for next sample
    while (micros() < nextSampleTime); // Wait until it's time for the next sample
  }

  unsigned long elapsedTime = micros() - startTime;                      // Calculate elapsed time in microseconds
  float actualSamplingFrequency = (1000000.0f * SAMPLES) / elapsedTime;  // Compute effective sampling frequency
  Serial.print("Actual Sampling Frequency: ");
  Serial.println(actualSamplingFrequency);

  // Perform FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Send FFT data for visualization
  Serial.print("<FFT>");                   // Start marker
  for (int i = 0; i < SAMPLES / 2; i++) {  // Only send first half of FFT (positive frequencies)
    Serial.print(vReal[i]);
    if (i < SAMPLES / 2 - 1) {
      Serial.print(",");  // Separate values with commas
    }
  }
  Serial.println("</FFT>");  // End marker
  Serial.println();

  // Find the most prevalent frequency
  const uint16_t peakFrequency = round(FFT.majorPeak());

  // Output result
  Serial.print("<PEAK>");
  Serial.print(peakFrequency);
  Serial.println("</PEAK>");
  Serial.println();


  // If frequency detected from pressure sensor is below 100 Hz, ignore
  if (peakFrequency > 100) {
    // Compute output frequency with valve input and pressure sensor FFT result
    bool valves[3];  // Array to store valve combination
    for (uint8_t i = 0; i < 3; i++) {
      valves[i] = digitalRead(valvePins[i]);
    }

    const uint8_t numSemitone = 2 * valves[0] + valves[1] + 3 * valves[2];  // Calculate the number of semitones to go down from BASE_FREQUENCY
    const uint16_t outputFrequency = computeOutput(peakFrequency, numSemitone);

    // Update piezo only if frequency changes
    if (outputFrequency != currentFrequency) {
      currentFrequency = outputFrequency;
      tone(piezo, currentFrequency);
    }
  } else {
    noTone(piezo);
    currentFrequency = 0;
  }
}

// Returns the frequency closest to that obtained from the FFT
const uint16_t computeOutput(const uint16_t fftFrequency, const uint8_t harmonicSeries) {

  uint16_t minDiff = abs(fftFrequency - frequencyChart[harmonicSeries][0]);
  uint16_t outputFreq = frequencyChart[harmonicSeries][0];

  for (uint8_t i = 1; i < 7; i++) {
    uint16_t diff = abs(fftFrequency - frequencyChart[harmonicSeries][i]);
    if (diff < minDiff) {
      minDiff = diff;
      outputFreq = frequencyChart[harmonicSeries][i];
    }
  }

  return outputFreq;
}