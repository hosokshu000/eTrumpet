#include <Arduino.h>
#include <I2S.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include "PressureSensor_rpPico.h"

#define SAMPLES 256                 // Must be a power of 2
#define SAMPLING_FREQUENCY 4500.0f  // FFT sampling frequency in Hz

#define SINE_TABLE_SIZE 256
#define SAMPLE_RATE 44100      // Sampling rate
#define MAX_VOLUME 0.01    // Keep this between 0.001 and 0.014 ALWAYS
#define MAX_HARMONICS 6  // Number of harmonics to add (the length of the harmonics array)

// Pressure sensor I2C pins
const uint8_t SDA_PIN = 14;
const uint8_t SCL_PIN = 15;

// FFT arrays
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Calculate FFT sampling period in microseconds
const unsigned long SAMPLING_PERIOD_US = round(1000000.0f / SAMPLING_FREQUENCY);

// Buffer for storing timestamp of each sample to track actual frequency
unsigned long sampleTimestamps[SAMPLES];


// I2S Variables
const int i2sDataPin = 20;    // I2S data pin (DOUT)
const int i2sClockPin = 18;   // I2S bit clock pin (BCLK)
const int i2sLRPin = 19;      // I2S word select pin (WS)

const float harmonics[] = {1.0, 0.5, 0.3, 0.2, 0.15, 0.1}; // Weights for each harmonic

uint16_t sineTable[SINE_TABLE_SIZE];
volatile float currentPhase = 0.0;
volatile float phaseIncrement = 0.0;
float volume = MAX_VOLUME;

// Store frequencies
volatile float currentFreq = 0;
volatile float prevFreq = 0;
volatile float detectedFreq = 0; // FFT output

// Flag to communicate frequency change between loops
volatile bool changeFreq = false;

// Audio output variables
I2S i2s(OUTPUT, i2sClockPin, i2sDataPin);
unsigned long previousMicros = 0;
const unsigned long sampleInterval = 1000000 / SAMPLE_RATE; 

uint8_t numSemitones = 0;

const uint8_t valvePins[] = {13, 12, 11}; // Buttons for the valves

// Trumpet harmonic series
const float frequencyChart[7][7] = {
  {233.08, 349.23, 466.16, 587.33, 698.46, 830.61, 932.33}, // C  Harmonic
  {220.00, 329.63, 440.00, 554.37, 659.25, 783.99, 880.00}, // B  Harmonic
  {207.65, 311.13, 415.30, 523.25, 622.25, 739.99, 830.61}, // Bb Harmonic
  {196.00, 293.66, 392.00, 493.88, 587.33, 698.46, 783.99}, // A. Harmonic
  {185.00, 277.18, 369.99, 466.16, 554.37, 659.25, 739.99}, // Ab Harmonic
  {174.61, 261.63, 349.23, 440.00, 523.25, 622.25, 698.46}, // G  Harmonic
  {164.81, 246.94, 329.63, 415.30, 493.88, 587.33, 659.25}  // Gb Harmonic
};

// Debugging actual sampling frequency of FFT
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

// Returns the frequency closest to that obtained from the FFT
float computeOutput(float fftFrequency, int harmonicSeries) {
  float minDiff = abs(fftFrequency - frequencyChart[harmonicSeries][0]);
  float outputFreq = frequencyChart[harmonicSeries][0];

  for (int i = 1; i < 7; i++) {
    float diff = abs(fftFrequency - frequencyChart[harmonicSeries][i]);
    if (diff < minDiff) {
      minDiff = diff;
      outputFreq = frequencyChart[harmonicSeries][i];
    }
  }

  return outputFreq;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wait for serial to initialize
  }
  
  Serial.println("Core 0 activated");

  delay(1000);

  // Confirm successful I2S connection
  if (!i2s.begin(SAMPLE_RATE)) {
    Serial.println("I2S Init Failed!");
    while (1);
  }

  for (int i = 0; i < 3; i++) {
    pinMode(valvePins[i], INPUT);
  }

  // Generate trumpet-like wave with harmonics
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    float sample = 0;
    for (int h = 0; h < MAX_HARMONICS; h++) {
      sample += harmonics[h] * sin(2.0 * PI * (h + 1) * i / SINE_TABLE_SIZE);
    }
    sineTable[i] = (uint16_t)((32767.5 * (1.0 + sample / 2.0)));
  }
  
  // Initialize I2C for pressure sensor on core 0
  initialize(SDA_PIN, SCL_PIN);

  phaseIncrement = (float)SINE_TABLE_SIZE * 440 / SAMPLE_RATE;
}

void setup1() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wait for serial to initialize
  }
  
  Serial.println("Core 1 activated!");
  delay(1000);
}

// I2S output on Core 0
void loop() {
  // Check flag and update frequency
  if (changeFreq) {
    currentFreq = detectedFreq;
    changeFreq = false;

    Serial.print("Frequency changed to: ");
    Serial.println(currentFreq);

    // Update phase increment if frequency changed
    if (prevFreq != currentFreq) {
      phaseIncrement = (float)SINE_TABLE_SIZE * currentFreq / SAMPLE_RATE;
      Serial.print("phaseIncrement changed to: ");
      Serial.println(phaseIncrement);
    }
  }
  
  // Audio generation timing
  unsigned long currentMicros = micros();
  
  if (currentMicros - previousMicros >= sampleInterval && currentFreq != 0) {
    previousMicros += sampleInterval;  // Maintain precise timing

    // Calculate sine value for current phase
    uint16_t sineValue = sineTable[(int)currentPhase];
    uint16_t scaledValue = sineValue * volume;

    // Output audio sample
    i2s.write(scaledValue);
    i2s.write(scaledValue);

    // Update phase (ensure smooth phase wraparound)
    currentPhase = fmod(currentPhase + phaseIncrement, SINE_TABLE_SIZE);
  }

  prevFreq = currentFreq;
}

// FFT loop running on Core 1
void loop1() {
  // Core 1: FFT analysis and frequency computation
  
  // Disable interrupts during sampling to enhance precision
  noInterrupts();
  
  unsigned long startTime = micros();
  unsigned long nextSampleTime = startTime;

  // Populate FFT array
  for (int i = 0; i < SAMPLES; i++) {
    // Wait until next sample time
    while (micros() < nextSampleTime) {}

    sampleTimestamps[i] = micros(); // Record debugging timestamp
        
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

  detectedFreq = 0; // Initially set frequency to 0
  numSemitones = 2 * digitalRead(valvePins[0]) + digitalRead(valvePins[1]) + 3 * digitalRead(valvePins[2]); // Calculate the number of semitones to go down from FFT.majorPeak()
  
  // Ignore all frequencies (noise) below 100 Hz
  if (FFT.majorPeak() > 100) {
    detectedFreq = computeOutput(FFT.majorPeak(), numSemitones);
  }

  changeFreq = true;
}