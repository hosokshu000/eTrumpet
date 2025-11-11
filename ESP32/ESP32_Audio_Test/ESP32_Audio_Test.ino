#include <Arduino.h>
#include "driver/i2s.h"

// I2S Configuration
#define SAMPLE_RATE 44100
#define SINE_TABLE_SIZE 256
#define TEST_FREQUENCY 440  // 1kHz test tone
#define VOLUME 0.002

const int i2sClockPin = 3;   // BCLK
const int i2sLRPin = 4;     // LRC (WS)
const int i2sDataPin = 2;    // DIN (Data In)

int16_t sineTable[SINE_TABLE_SIZE];

const float harmonics[] = {1.0, 0.5, 0.3, 0.2, 0.15, 0.1};

// Audio generation variables
float currentPhase = 0.0;
float phaseIncrement = 0.0;

void generateSineTable() {
  Serial.println("Generating sine table...");

  // Precompute total harmonic weight
  float totalWeight = 0;
  for (int h = 0; h < 6; h++) totalWeight += harmonics[h];

  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    float sample = 0;
    for (int h = 0; h < 6; h++) {
      sample += harmonics[h] * sin(2.0 * PI * (h + 1) * i / SINE_TABLE_SIZE);
    }
    // Normalize combined amplitude before scaling by volume
    sample /= totalWeight;

    sineTable[i] = (int16_t)(sample * 32767.0 * VOLUME);
  }
}

void setupI2S() {
  Serial.println("Setting up I2S for MAX98357A...");
  
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = true,   
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  // I2S pin configuration
  i2s_pin_config_t pin_config = {
    .bck_io_num = i2sClockPin,
    .ws_io_num = i2sLRPin,
    .data_out_num = i2sDataPin,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  // Install and start I2S driver
  esp_err_t result = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (result != ESP_OK) {
    Serial.print("Failed to install I2S driver: ");
    Serial.println(result);
    return;
  }

  result = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (result != ESP_OK) {
    Serial.print("Failed to set I2S pins: ");
    Serial.println(result);
    return;
  }

  // Clear I2S buffer
  i2s_zero_dma_buffer(I2S_NUM_0);

  Serial.println("I2S setup complete!");
  Serial.print("BCLK Pin: "); Serial.println(i2sClockPin);
  Serial.print("LRCLK Pin: "); Serial.println(i2sLRPin);
  Serial.print("DIN Pin: "); Serial.println(i2sDataPin);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== ESP32-S3 I2S Audio Test for MAX98357A ===");
  Serial.print("Test Frequency: "); Serial.print(TEST_FREQUENCY); Serial.println(" Hz");
  Serial.print("Sample Rate: "); Serial.print(SAMPLE_RATE); Serial.println(" Hz");
  Serial.print("Volume: "); Serial.print(VOLUME * 100); Serial.println("%");
  
  generateSineTable();
  
  // Setup I2S
  setupI2S();
  
  // Calculate phase increment for desired frequency
  phaseIncrement = (float)SINE_TABLE_SIZE * TEST_FREQUENCY / SAMPLE_RATE;
  Serial.print("Phase increment: "); Serial.println(phaseIncrement);
  
  Serial.println("Starting audio output...");
  Serial.print("You should hear a "); Serial.print(TEST_FREQUENCY); Serial.println("Hz tone at low volume.");
  Serial.println("Send any character via Serial to stop.");
}

void loop() {
  // Check for serial input to stop
  if (Serial.available()) {
    Serial.println("Stopping audio...");
    i2s_driver_uninstall(I2S_NUM_0);
    while(1) delay(1000);
  }
  
  // Get sine value from lookup table
  int phaseIndex = (int)currentPhase % SINE_TABLE_SIZE;
  int16_t sampleValue = sineTable[phaseIndex];
  
  // Update phase for next sample
  currentPhase += phaseIncrement;
  if (currentPhase >= SINE_TABLE_SIZE) {
    currentPhase -= SINE_TABLE_SIZE;
  }
  
  size_t bytesWritten;
  esp_err_t result = i2s_write(I2S_NUM_0, &sampleValue, sizeof(sampleValue), &bytesWritten, portMAX_DELAY);
  
  if (result != ESP_OK) {
    Serial.print("I2S write error: ");
    Serial.println(result);
  }
}