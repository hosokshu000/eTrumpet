#include <Arduino.h>
#include <arduinoFFT.h>
#include "PressureSensor_rpPico.h"
#include <Wire.h>

#define SAMPLES 256                 // Must be a power of 2
#define SAMPLING_FREQUENCY 3000.0f  // Sampling frequency in Hz

// Pin setup
const uint8_t valvePins[] = {13, 12, 11};
const uint8_t PIEZO = 10;

// I2C pins
const uint8_t SDA_PIN = 14;
const uint8_t SCL_PIN = 15;

// Trumpet harmonic series
const uint16_t frequencyChart[7][7] = {
  {233, 349, 466, 587, 698, 831, 932},
  {220, 330, 440, 554, 659, 784, 880},
  {208, 311, 415, 523, 622, 740, 831},
  {196, 294, 392, 494, 587, 698, 784},
  {185, 277, 370, 466, 554, 659, 740},
  {175, 262, 349, 440, 523, 622, 698},
  {165, 247, 330, 415, 494, 587, 659}
};

// FFT arrays
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Calculate sampling period in microseconds
const unsigned long SAMPLING_PERIOD_US = round(1000000.0f / SAMPLING_FREQUENCY);

// Buffer for storing timestamp of each sample
unsigned long sampleTimestamps[SAMPLES];

// Track current frequency to avoid redundant tone updates
int currentFrequency = 0;

void setup() {
  for (int i = 0; i < 3; i++) {
    pinMode(valvePins[i], INPUT);
  }
  pinMode(PIEZO, OUTPUT);
  initialize(SDA_PIN, SCL_PIN); // Initialize I2C

  Serial.begin(115200);
  
  // Wait for serial to be ready
  while (!Serial) {
    delay(10);
  }
}

void loop() {
  // Disable interrupts during sampling to enhance precision
  noInterrupts();
  
  unsigned long startTime = micros();
  unsigned long nextSampleTime = startTime;

  // Populate FFT array
  for (int i = 0; i < SAMPLES; i++) {
    // Wait until next sample time
    while (micros() < nextSampleTime) {
      asm volatile ("nop\n\t");
    }
    
    // Record actual timestamp
    sampleTimestamps[i] = micros();
    
    // Read sensor and store value
    vReal[i] = getPressure();
    vImag[i] = 0;
    
    // Calculate next sample time
    nextSampleTime = startTime + (i + 1) * SAMPLING_PERIOD_US;
  }
  
  // Re-enable interrupts
  interrupts();

  // Print timing statistics
  //printTimingStats();

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
  uint16_t peakFrequency = round(FFT.majorPeak());
  Serial.print("<PEAK>");
  Serial.print(peakFrequency);
  Serial.println("</PEAK>");

  // If frequency detected from pressure sensor is below 100 Hz, ignore
  if (peakFrequency > 100) {
    // Compute output frequency with valve input and pressure sensor FFT result
    bool valves[3]; // Array to store valve combination
    for (int i = 0; i < 3; i++) {
      valves[i] = digitalRead(valvePins[i]);
    }

    const uint8_t numSemitone = 2 * valves[0] + valves[1] + 3 * valves[2]; // Calculate the number of semitones to go down from BASE_FREQUENCY
    const int outputFrequency = computeOutput(peakFrequency, numSemitone);

    // Update piezo only if frequency changes
    if (outputFrequency != currentFrequency) {
      currentFrequency = outputFrequency;
      play(currentFrequency);
    }
  }
  else {
    play(0);
    currentFrequency = 0;
  }

  delay(50);
}

// Returns the frequency closest to that obtained from the FFT
const int computeOutput(const int fftFrequency, const int harmonicSeries) {

  uint16_t minDiff = abs(fftFrequency - frequencyChart[harmonicSeries][0]);
  int outputFreq = frequencyChart[harmonicSeries][0];

  for (int i = 1; i < 7; i++) {
    int diff = abs(fftFrequency - frequencyChart[harmonicSeries][i]);
    if (diff < minDiff) {
      minDiff = diff;
      outputFreq = frequencyChart[harmonicSeries][i];
    }
  }

  return outputFreq;
}

void play(int frequency) {
  if (frequency == 0) {
    analogWriteFreq(0);
    analogWrite(PIEZO, 0);
  }
  else {
    analogWriteFreq(frequency);
    analogWrite(PIEZO, 128);
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