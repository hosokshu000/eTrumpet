// Test code for I2S audio

#include <Arduino.h>
#include <I2S.h>

const int i2sDataPin = 20;    // I2S data pin (DOUT)
const int i2sClockPin = 18;   // I2S bit clock pin (BCLK)
const int i2sLRPin = 19;      // I2S word select pin (WS)

#define SINE_TABLE_SIZE 256   
#define SAMPLE_RATE 44100      // Sampling rate
#define DEFAULT_FREQ 698       // A4 note
#define MAX_VOLUME 0.003    // Keep this between 0.001 and 0.014 ALWAYS

const float harmonics[] = {1.0, 0.5, 0.3, 0.2, 0.15, 0.1}; // Weights for overtones

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

uint16_t sineTable[SINE_TABLE_SIZE]; // Waveform array
float currentPhase = 0.0;
float phaseIncrement = 0.0;
float volume = MAX_VOLUME;

unsigned long previousMicros = 0;
const unsigned long sampleInterval = 1000000 / SAMPLE_RATE;  

I2S i2s(OUTPUT, i2sClockPin, i2sDataPin);

void i2sSetup() {
  // Generate trumpet-like wave with harmonics
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    float sample = 0;
    for (int h = 0; h < 6; h++) {
      sample += harmonics[h] * sin(2.0 * PI * (h + 1) * i / SINE_TABLE_SIZE);
    }
    sineTable[i] = (uint16_t)((32767.5 * (1.0 + sample / 2.0)));
  }

  // Correctly compute phase increment per sample
  phaseIncrement = (float)SINE_TABLE_SIZE * DEFAULT_FREQ / SAMPLE_RATE;
}

void loop() {
  unsigned long currentMicros = micros();
  
  if (currentMicros - previousMicros >= sampleInterval) {
    previousMicros += sampleInterval;  // Maintain precise timing

    uint16_t sineValue = sineTable[(int)currentPhase];

    uint16_t scaledValue = sineValue * volume;

    i2s.write(scaledValue);
    i2s.write(scaledValue);

    // Ensure smooth phase wraparound
    currentPhase = fmod(currentPhase + phaseIncrement, SINE_TABLE_SIZE);
  }
}