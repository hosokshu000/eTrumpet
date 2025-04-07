#ifndef PRESSURE_SENSOR_RP
#define PRESSURE_SENSOR_RP

#include <Wire.h>

#define SENSOR_ADDRESS 0x28

#define P_MAX 5.0f // Maximum pressure 5 psi
#define P_MIN -5.0f // Minimum pressure -5 psi
#define COUNTS_14BIT 16384  // Maximum bit size 2^14

#define O_MIN (COUNTS_14BIT * 0.1f)  // Minimum output (10% of 2^14)
#define O_MAX (COUNTS_14BIT * 0.9f)  // Maximum output (90% of 2^14)

// Initialize I2C
int initialize(const uint8_t SDA, const uint8_t SCL) {  
  Wire1.setSDA(SDA);
  Wire1.setSCL(SCL);
  Wire1.setClock(400000);
  Wire1.begin();

  return 1;
}

// Bits to psi conversion
float transferFunc(uint16_t bitOut) {
  return ((float)(bitOut - O_MIN) / (O_MAX - O_MIN)) * (P_MAX - P_MIN) + P_MIN;
}

float getPressure() {
  Wire1.beginTransmission(SENSOR_ADDRESS);
  Wire1.endTransmission();
  
  Wire1.requestFrom(SENSOR_ADDRESS, 2);

  if (Wire1.available() >= 2) {
    uint8_t msb = Wire1.read();
    uint8_t lsb = Wire1.read();
    
    // Mask out status bits (bits 7-6) from MSB
    msb &= 0x3F;
    
    // Combine bytes: MSB contains bits 13-8, LSB contains bits 7-0
    uint16_t output = ((uint16_t)msb << 8) | lsb;
    
    return transferFunc(output);
  }
  return 0;
}
#endif