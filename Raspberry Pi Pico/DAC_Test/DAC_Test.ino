// Note Player for RP2040 Pico 2 with MAX98357A DAC using Arduino IDE
// For use with TPA3118 amplifier and speaker

#include <I2S.h>

// Define pins for I2S connection to MAX98357A
const int I2S_BCLK = 18;  // Bit Clock
const int I2S_LRCLK = 19; // LR Clock (Word Select)
const int I2S_DOUT = 20;  // Data Out from Pico to MAX98357A

// Audio configuration
const int SAMPLE_RATE = 44100;
const int BUFFER_SIZE = 128;
int16_t audio_buffer[BUFFER_SIZE];

// Note frequencies (in Hz)
const float NOTE_C4 = 261.63f;
const float NOTE_CS4 = 277.18f;
const float NOTE_D4 = 293.66f;
const float NOTE_DS4 = 311.13f;
const float NOTE_E4 = 329.63f;
const float NOTE_F4 = 349.23f;
const float NOTE_FS4 = 369.99f;
const float NOTE_G4 = 392.00f;
const float NOTE_GS4 = 415.30f;
const float NOTE_A4 = 440.00f;
const float NOTE_AS4 = 466.16f;
const float NOTE_B4 = 493.88f;

// Variables for note generation
float current_note = NOTE_A4;
uint32_t note_duration_ms = 1000; // 1 second per note
uint32_t note_start_time = 0;
float phase = 0.0f;
float volume = 0.7f; // Volume level (0.0 to 1.0)

// Define a simple melody
float melody[] = {
  NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C4, 
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_G4,
  NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C4
};
int melody_length = sizeof(melody) / sizeof(melody[0]);
int current_melody_index = 0;

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to connect
  Serial.println("Rp Pico 2 Note Player Starting...");

  // Initialize I2S
  if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 16)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // Don't proceed if I2S initialization failed
  }
  
  // Configure I2S pins
  I2S.setAllPins(-1, -1, I2S_DOUT, I2S_BCLK, I2S_LRCLK);
  
  Serial.println("I2S initialized successfully");
  note_start_time = millis();
}

void loop() {
  // Check if it's time to change to the next note
  if (millis() - note_start_time >= note_duration_ms) {
    // Move to next note in melody
    current_melody_index = (current_melody_index + 1) % melody_length;
    current_note = melody[current_melody_index];
    
    note_start_time = millis();
    Serial.print("Playing note: ");
    Serial.println(current_note);
  }
  
  // Generate audio data and send it to I2S
  generateAndPlayAudio();
}

void generateAndPlayAudio() {
  // Calculate how far into the note we are (0.0 to 1.0)
  float note_progress = (float)(millis() - note_start_time) / note_duration_ms;
  
  // Fill the audio buffer with sine wave samples
  for (int i = 0; i < BUFFER_SIZE; i++) {
    // Generate sine wave
    float phase_increment = 2.0f * PI * current_note / SAMPLE_RATE;
    float sample = sin(phase);
    
    // Apply ADSR envelope
    float envelope = applyEnvelope(note_progress);
    
    // Apply volume and envelope to sample
    int16_t value = (int16_t)(32767.0f * sample * volume * envelope);
    audio_buffer[i] = value;
    
    // Increment phase for next sample
    phase += phase_increment;
    if (phase > 2.0f * PI) {
      phase -= 2.0f * PI;
    }
  }
  
  // Send buffer to I2S
  I2S.write(audio_buffer, BUFFER_SIZE);
}

float applyEnvelope(float note_progress) {
  // Simple ADSR envelope
  // Attack: 0-10% of note duration
  // Decay: 10-20% of note duration
  // Sustain: 20-80% of note duration
  // Release: 80-100% of note duration
  
  if (note_progress < 0.1f) {
    // Attack - ramp up
    return note_progress / 0.1f;
  } else if (note_progress < 0.2f) {
    // Decay - slight fall to sustain level
    float decay_progress = (note_progress - 0.1f) / 0.1f;
    return 1.0f - (0.2f * decay_progress);
  } else if (note_progress < 0.8f) {
    // Sustain - constant level
    return 0.8f;
  } else if (note_progress < 1.0f) {
    // Release - fall to zero
    float release_progress = (note_progress - 0.8f) / 0.2f;
    return 0.8f * (1.0f - release_progress);
  } else {
    // Beyond note duration
    return 0.0f;
  }
}