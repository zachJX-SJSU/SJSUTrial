#include <Servo.h>
#include <Wire.h>

/* Set up */
// ---- Servo Setup ----
Servo servo_x;
Servo servo_y;
int SERV_PIN_X = 9;
int SERV_PIN_Y = 11;
int angle = 0;

// ---- MPU setup ----
#define MPU_ADDR      0x68   // Default MPU6050 address (AD0=GND → 0x68, AD0=VCC → 0x69)
#define REG_PWR_MGMT1 0x6B   // Register: power management (controls sleep/wake)
#define REG_ACCEL_XH  0x3B   // First register of the accelerometer output (14 bytes span Ax..Gz)

const float ACC_LSB_PER_G    = 16384.0f;  // accelerometer: 16384 LSB = 1 g
/* End of set up */

// Forward-declaring helper functions

/* Helper functions for MPU communication, since MPU library is not allowed
  combine two 8-bit registers into one signed 16-bit value */
static inline int16_t make16(uint8_t hi, uint8_t lo) { 
  return (int16_t)((hi << 8) | lo); 
}

// ---- I²C helper functions ----
void i2cWrite8(uint8_t reg, uint8_t val) {
  // Write one byte (val) into register (reg) of MPU6050
  Wire.beginTransmission(MPU_ADDR); // Start communication with MPU
  Wire.write(reg);                  // Tell it which register
  Wire.write(val);                  // Provide the value to write
  Wire.endTransmission(true);       // End transmission and release the bus
}

void i2cReadN(uint8_t startReg, uint8_t *buf, uint8_t n) {
  // Read multiple bytes sequentially starting at startReg
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);             // Start address
  Wire.endTransmission(false);      // Repeated start
  Wire.requestFrom(MPU_ADDR, n, true); // Request n bytes
  for (uint8_t i = 0; i < n && Wire.available(); i++) {
    buf[i] = Wire.read();           // Store into buffer
  }
}
/* End of Helper functions*/

void setup() {
  // Set up X,Y servos
  servo_x.attach(SERV_PIN_X); // Pin 9
  servo_y.attach(SERV_PIN_Y); // Pin 11

  Serial.begin(9600);     // May change to 115200 baud for faster rate
  Wire.begin();           // Start I²C as master on SDA/SCL pins of Uno R4

  i2cWrite8(REG_PWR_MGMT1, 0x00); // Wake up the MPU6050 by clearing the sleep bit
  delay(50);                      // Small delay to stabilize
}

void loop() {
  uint8_t b[14];                     // Buffer to hold 14 bytes from MPU
  i2cReadN(REG_ACCEL_XH, b, 14);     // Read accel (6B), temp (2B), gyro (6B)

  // Only care about accelerator data, not gyro data
  int16_t rawAx = make16(b[0],  b[1]);
  int16_t rawAy = make16(b[2],  b[3]);
  int16_t rawAz = make16(b[4],  b[5]);

  // Convert raw data into physical units
  float Ax_g = rawAx / ACC_LSB_PER_G;   // acceleration in g
  float Ay_g = rawAy / ACC_LSB_PER_G;
  float Az_g = rawAz / ACC_LSB_PER_G;

  // Drive motor to the right x/y angle
  int xAngle = map(rawAx,-16384, 16384, 0, 180);
  int yAngle = map(rawAy,-16384, 16384, 0, 180);

  servo_x.write(xAngle);
  servo_y.write(yAngle);

  // Print results over serial
  Serial.print("Accel[g]  X: "); Serial.print(Ax_g, 3); //Serial.print(rawAx, DEC);
  Serial.print("  Y: ");        Serial.print(Ay_g, 3);
  Serial.print("  Z: ");        Serial.print(Az_g, 3);
  Serial.println(xAngle);

  delay(100);  // Pause 100 ms before next read
}
