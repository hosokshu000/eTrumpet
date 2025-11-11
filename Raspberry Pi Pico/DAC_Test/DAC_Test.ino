// Test code for I2S audio
#include <Arduino.h>
#include <I2S.h>

const int i2sDataPin = ;    // I2S data pin (DOUT)
const int i2sClockPin = 18;   // I2S bit clock pin (BCLK)
const int i2sLRPin = 19;      // I2S word select pin (WS)

#define SINE_TABLE_SIZE 256   
#define SAMPLE_RATE 44100      // Sampling rate
#define DEFAULT_FREQ 440       
#define MAX_VOLUME 0.002    // Keep this between 0.001 and 0.014 ALWAYS

const float harmonics[] = {1.0, 0.5, 0.3, 0.2, 0.15, 0.1};

uint16_t sineTable[SINE_TABLE_SIZE];
float currentPhase = 0.0;
float phaseIncrement = 0.0;
float volume = MAX_VOLUME;

unsigned long previousMicros = 0;
const unsigned long sampleInterval = 1000000 / SAMPLE_RATE;  

I2S i2s(OUTPUT, i2sClockPin, i2sDataPin);

void setup() {
  Serial.begin(115200);
  
  if (!i2s.begin(SAMPLE_RATE)) {
    Serial.println("I2S Init Failed!");
    while (1);
  }

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