#include <Wire.h>

#define SENSOR_ADDRESS 0x28

#define P_MAX 5.0f // Maximum pressure 5 psi
#define P_MIN -5.0f // Minimum pressure -5 psi
#define COUNTS_14BIT 16384  // Maximum bit size 2^14

#define O_MIN (COUNTS_14BIT * 0.1f)  // Minimum output (10% of 2^14)
#define O_MAX (COUNTS_14BIT * 0.9f)  // Maximum output (90% of 2^14)

// Bits to psi
float transferFunc(uint16_t bitOut) {
  return ((float)(bitOut - O_MIN) / (O_MAX - O_MIN)) * (P_MAX - P_MIN) + P_MIN;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
}

void loop() {
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.endTransmission();
  
  Wire.requestFrom(SENSOR_ADDRESS, 2);

  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    
    // Mask out status bits (bits 7-6) from MSB
    msb &= 0x3F;
    
    // Combine bytes: MSB contains bits 13-8, LSB contains bits 7-0
    uint16_t output = ((uint16_t)msb << 8) | lsb;
    
    Serial.println(transferFunc(output));
  }
}