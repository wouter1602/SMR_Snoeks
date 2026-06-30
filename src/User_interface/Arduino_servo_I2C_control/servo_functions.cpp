
#include <Wire.h>
#include "servo_functions.h"

Adafruit_PWMServoDriver boards[NUM_BOARDS] = {
    Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS_1),
    Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS_2),
    Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS_3)
  };

inline void resolveServo(uint8_t index, uint8_t &board, uint8_t &channel) {
  board = index / CHANNELS_PER_BOARD;
  channel = index % CHANNELS_PER_BOARD;
}

// convert servo degrees to puls with and set it
void setServoAngle(uint8_t index, uint8_t angleDeg) {
  if (index >= NUM_SERVOS) return;
  angleDeg = constrain(angleDeg, 0, 180);

  uint16_t pulse = map(angleDeg, 0, 180, SERVO_MIN_COUNT, SERVO_MAX_COUNT);

  uint8_t board, channel;
  resolveServo(index, board, channel);
  boards[board].setPWM(channel, 0, pulse);
}

// Detach (stop driving) a single servo - let it go limp.
void releaseServo(uint8_t index) {
  uint8_t board, channel;
  resolveServo(index, board, channel);
  boards[board].setPWM(channel, 0, 0);
}

void setupServos() {
  for (uint8_t i = 0; i < NUM_BOARDS; i++) {
    boards[i].begin();
    boards[i].setOscillatorFrequency(27000000); // Set internal clock to 27 MHz
    boards[i].setPWMFreq(SERVO_FREQ_HZ);
    delay(10);  // Wait for setup so not all start at the same time
  }

  // Soft-start: bring servos to zero one at a time. This avoids a 48-servo simultaneaous inrush burnout.
  sendLog("Staring servos...");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, 90);
    delay(25); //staggered
  }
}