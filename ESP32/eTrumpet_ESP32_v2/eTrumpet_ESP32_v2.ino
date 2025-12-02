#include <Arduino.h>
#include "driver/i2s.h"
#include <Wire.h>
#include <arduinoFFT.h>
#include "PressureSensor_ESP32.h"
#include "Waveform.h"


// FFT Macros
#define SAMPLES 128
#define SAMPLING_FREQUENCY 5000.0f
#define MIN_AMPLITUDE 0.30
#define MAX_AMPLITUDE 2.00
#define PRESSURE_BIAS 0.0105 // Pressure sensor bias


// I2S Macros
#define SINE_TABLE_SIZE 1024
#define SAMPLE_RATE 44100
#define MAX_VOLUME 0.026 // Tested up to 0.028, where there's a bit of distortion. Limit is probably 0.030...? But 0.026 is already pretty loud
#define MIN_VOLUME 0.002
#define BUFFER_LEN 256


// Thresholds
#define VALVE_PRESS 2300 // Hall effect sensor threshold for valve on/off
#define PITCH_CHANGE_REQ 1 // Number of consecutive frequency readings required to change pitch (increasing this increases "resistance" of instrument)
#define OVERTONE_THRES 2.9 // Fundamental x 2 / Fundamental must be over this threshold for the fundamental x 2 frequency to be considered the fundamental


// GPIO Setup
const int potPin = 1;
const uint8_t valvePins[] = {7, 8, 9};
const uint8_t SDA_PIN = 5;
const uint8_t SCL_PIN = 6;
const int i2sLRPin = 4; // LRC on DAC
const int i2sClockPin = 3; // BCLK on DAC
const int i2sDataPin = 2; // DIN on DAC


// FFT Variables
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
const unsigned long SAMPLING_PERIOD_US = round(1000000.0f / SAMPLING_FREQUENCY);
unsigned long sampleTimestamps[SAMPLES];


// Shared variables (protected by mutex)
volatile float detectedFreq = 0;
volatile float amplitude = 0;
volatile bool changeFreq = false;


// Wave Generation Variables
uint16_t sineTable[SINE_TABLE_SIZE];
volatile float currentPhase = 0.0;
volatile float phaseIncrement = 0.0;
const unsigned long sampleInterval = 1000000 / SAMPLE_RATE;


// Frequency Variables
float currentFreq = 0;
float prevFreq = 0;
float defaultFreq = (float)SINE_TABLE_SIZE * 440 / SAMPLE_RATE;


// Audio output variables
uint8_t numSemitones = 0;
float volume = 0;


// Trumpet harmonic series
const float frequencyChart[7][7] = {
  {233.08, 349.23, 466.16, 587.33, 698.46, 830.61, 932.33},
  {220.00, 329.63, 440.00, 554.37, 659.25, 783.99, 880.00},
  {207.65, 311.13, 415.30, 523.25, 622.25, 739.99, 830.61},
  {196.00, 293.66, 392.00, 493.88, 587.33, 698.46, 783.99},
  {185.00, 277.18, 369.99, 466.16, 554.37, 659.25, 739.99},
  {174.61, 261.63, 349.23, 440.00, 523.25, 622.25, 698.46},
  {164.81, 246.94, 329.63, 415.30, 493.88, 587.33, 659.25}
};


// Pressure data filtering variables
int pitchChangeCount = 0;
bool filteringPitch = false;
float originalVolume = 0;
float prevVolume = 0;
float originalFreq = 0;
float candidateFreq = 0;


// Debug timer
unsigned long lastHarmonicPrint = 0;


// Potentiometer parameter
int param = 0;
int prevParam = 0;


// Prevent value corruption between two cores
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


// Determines partial from FFT output by finding lowest supported note in harmonic series
float* getTargetFreqAndAmp(int harmonicSeries) {
  static float out[2];

  if (harmonicSeries < 0 || harmonicSeries > 6) {
    out[0] = 0.0f;
    out[1] = 0.0f;
    return out;
  }

  // Compute a noise floor using upper FFT bins
  float noiseSum = 0.0f;
  int noiseCount = 0;

  int startBin = (SAMPLES / 2) * 0.75;
  int endBin   = SAMPLES / 2;

  for (int b = startBin; b < endBin; b++) {
    noiseSum += vReal[b];
    noiseCount++;
  }

  float noiseFloor = (noiseCount > 0) ? (noiseSum / noiseCount) : 0.0f;

  float mainThreshold = max((float)MIN_AMPLITUDE, noiseFloor * 3.0f);
  float supportThreshold = max(0.001f, mainThreshold * 0.45f);

  int bins[7];
  float amps[7];

  for (int i = 0; i < 7; i++) {
    float f = frequencyChart[harmonicSeries][i];
    int bin = round(f * SAMPLES / SAMPLING_FREQUENCY);

    if (bin < 0) bin = 0;
    if (bin >= SAMPLES) bin = SAMPLES - 1;

    bins[i] = bin;
    amps[i] = vReal[bin];
  }

  int chosenIndex = -1;

  for (int m = 0; m < 7; m++) {
    if (amps[m] < mainThreshold) continue;  // Not strong enough alone

    // Look for supporting higher harmonic
    for (int h = m + 1; h < 7; h++) {
      if (amps[h] >= supportThreshold) {
        chosenIndex = m;
        break;
      }
    }

    if (chosenIndex >= 0) break;
  }

  if (chosenIndex < 0) {
    float bestAmp = -1.0f;
    int bestIdx = 0;

    for (int i = 0; i < 7; i++) {
      if (amps[i] > bestAmp) {
        bestAmp = amps[i];
        bestIdx = i;
      }
    }

    chosenIndex = bestIdx;
  }

  out[0] = frequencyChart[harmonicSeries][chosenIndex];
  out[1] = amps[chosenIndex];
  return out;
}


// Map FFT amplitude to volume
float ampToVol(float amplitude) {
  return MIN_VOLUME + (amplitude - MIN_AMPLITUDE) / (MAX_AMPLITUDE - MIN_AMPLITUDE) * (MAX_VOLUME - MIN_VOLUME);
}


// Return potentiometer reading as a volume scale factor between 0 and 1
float scalePot(int val) {
  return val / (float)4095;
}


// Waveform Generator
void populateSinTable(const float* harmonics) {
  int numHarmonics = (int)harmonics[0];

  // Precompute total harmonic weight
  float totalWeight = 0;
  for (int h = 1; h <= numHarmonics; h++) totalWeight += harmonics[h];

  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    float sample = 0;
    for (int h = 1; h <= numHarmonics; h++) { // Skip the number of harmonics stored in harmonics[0]
      sample += harmonics[h] * sin(2.0 * PI * h * i / SINE_TABLE_SIZE);
    }
    // Normalize combined amplitude before scaling by volume to ensure consistent volume
    sample /= totalWeight;

    sineTable[i] = (int16_t)(sample * 32767.0);
  }
}


// mode parameter - 0: Timbre, 1: Pitch Modulation, 2: Vibrato, 3: Change Octave
void checkPot(int mode, volatile float* freq) {
  if (mode == 0) {
    param = analogRead(potPin) / 2048;

    if (param != prevParam) {
      if (param < 1) {
        populateSinTable(tpt_enriched);
      }
      else {
        populateSinTable(woodwindHarmonics);
      }
    }
    prevParam = param;
  } else if (mode == 1) {
    param = analogRead(potPin);
    float depth = 0.03;  // Adjust for more/less pitch modulation
    float modAmount = *freq * depth * (float)param / 4095;
    *freq += modAmount;
    if (modAmount > 0) {
      changeFreq = true;
    }
  } else if (mode == 2) {

  } else if (mode == 3) {
    if (analogRead(potPin) / 2048 > 0) {
      *freq = 1046.50;
      changeFreq = true;
    }
  }
}


unsigned long lastDebugPrint = 0;

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


void printFFTForVisualizer() {
    static char fftBuf[2048];
    int idx = 0;

    idx += sprintf(fftBuf + idx, "<FFT>");

    for (int i = 0; i < SAMPLES / 2; i++) {
        idx += sprintf(fftBuf + idx, "%.4f", vReal[i]);
        if (i < SAMPLES / 2 - 1) {
            fftBuf[idx++] = ',';
        }
    }

    idx += sprintf(fftBuf + idx, "</FFT>\n");
    fftBuf[idx] = '\0';

    Serial.print(fftBuf);
}


void printDetectedPeak(float freq) {
    static char buf[32];
    int idx = 0;

    idx += sprintf(buf + idx, "<PEAK>");
    idx += sprintf(buf + idx, "%.1f", freq);
    idx += sprintf(buf + idx, "</PEAK>\n");

    Serial.print(buf);
}


void FFTtask(void* pvParameters) {
  while (true) {
    unsigned long startTime = micros();
    for (int i = 0; i < SAMPLES; i++) {
      unsigned long targetTime = startTime + (i * SAMPLING_PERIOD_US);
      while (micros() < targetTime) {
      }

      sampleTimestamps[i] = micros();
      vReal[i] = getPressure() - PRESSURE_BIAS;
      vImag[i] = 0;
    }

    // Perform FFT
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);

    // Compute the number of semitones to go down from the open harmonic based on the valve combination
    numSemitones = 2 * (analogRead(valvePins[0]) > VALVE_PRESS) + (analogRead(valvePins[1]) > VALVE_PRESS) + 3 * (analogRead(valvePins[2]) > VALVE_PRESS);

    /*unsigned long now = millis();
    if (now - lastHarmonicPrint >= 500) {   // 10 seconds = 10000 ms
        //printHarmonicAmplitudes(numSemitones);
        //Serial.println(FFT.majorPeak());
        lastHarmonicPrint = now;
    }*/

    float* pitchInfo = getTargetFreqAndAmp(numSemitones); // pitchInfo: {target frequency, amplitude}
    float localAmplitude = pitchInfo[1];
    float localDetectedFreq = 0;
    
    if (localAmplitude > MIN_AMPLITUDE) {
      localDetectedFreq = pitchInfo[0];
    }

    portENTER_CRITICAL(&mux);
    amplitude = localAmplitude;
    detectedFreq = localDetectedFreq;
    changeFreq = true;
    portEXIT_CRITICAL(&mux);
  }
}


void AudioTask(void* pvParameters) {
  int16_t sampleBuffer[BUFFER_LEN];

  while (true) {
    float localFreq = 0;
    float localAmplitude = 0;
    bool hasNewData = false;

    checkPot(1, &detectedFreq);
    
    portENTER_CRITICAL(&mux);
    if (changeFreq) {
      localFreq = detectedFreq;
      localAmplitude = amplitude;
      hasNewData = true;
      changeFreq = false;
    }
    portEXIT_CRITICAL(&mux);

    // Process frequency change
    if (hasNewData) {
      currentFreq = localFreq;
      
      if (currentFreq != prevFreq) {
        filteringPitch = true;
        originalFreq = prevFreq;
        originalVolume = prevVolume;
        candidateFreq = currentFreq;
        pitchChangeCount = 1;
      }

      if (filteringPitch) {
        if (currentFreq == candidateFreq) {
          pitchChangeCount++;
          if (pitchChangeCount >= PITCH_CHANGE_REQ) {
            filteringPitch = false;
            pitchChangeCount = 0;
          }
        } else {
          filteringPitch = false;
          pitchChangeCount = 0;
        }
      }

      // Update phase increment if frequency changed
      if (currentFreq != originalFreq && !filteringPitch) {
        phaseIncrement = (float)SINE_TABLE_SIZE * currentFreq / SAMPLE_RATE;
      }

      volume = min(ampToVol(localAmplitude), (float)MAX_VOLUME);
    }

    if (currentFreq <= 0.0f || volume <= 0.0f) {
      memset(sampleBuffer, 0, sizeof(sampleBuffer)); // fill with silence
    } else {
        for (size_t i = 0; i < BUFFER_LEN; i++) {
          int phaseIndex = (int)fmodf(currentPhase, SINE_TABLE_SIZE);
          if (phaseIndex < 0) phaseIndex += SINE_TABLE_SIZE;

          int16_t rawSample = (int16_t)sineTable[phaseIndex];
          int16_t scaledSample = (int16_t)(rawSample * volume);

          sampleBuffer[i] = scaledSample;

          currentPhase += phaseIncrement;
          if (currentPhase >= SINE_TABLE_SIZE) currentPhase -= SINE_TABLE_SIZE;
        }
    }

    // Write whole buffer to I2S in one go
    size_t bytesWritten;
    esp_err_t result = i2s_write(I2S_NUM_0, sampleBuffer, sizeof(sampleBuffer), &bytesWritten, portMAX_DELAY);
    if (result != ESP_OK) {
      Serial.print("I2S write error: ");
      Serial.println(result);
    }

    prevFreq = currentFreq;
    prevVolume = volume;
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize valve input
  for (int i = 0; i < 3; i++) {
    pinMode(valvePins[i], INPUT);
  }
  
  // Initialize I2C (Pressure Sensor)
  initialize(SDA_PIN, SCL_PIN);

  // Initialize volume knob input
  pinMode(potPin, INPUT);

  // Initialize I2S (Audio Generation)
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_LEN,
    .use_apll = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = i2sClockPin,
    .ws_io_num = i2sLRPin,
    .data_out_num = i2sDataPin,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // Initialize Waveform
  populateSinTable(tpt_enriched);

  phaseIncrement = defaultFreq;

  // FFT task on core 0
  xTaskCreatePinnedToCore(FFTtask, "FFTtask", 32768, NULL, 2, NULL, 0);

  // I2S generation on core 1
  xTaskCreatePinnedToCore(AudioTask, "AudioTask", 32768, NULL, 2, NULL, 1);
}


void loop() {
  // Required by Arduino IDE
}