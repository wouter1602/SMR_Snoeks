
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#define PCA9685_I2C_ADDRESS_1 PCA9685_I2C_ADDRESS
#define PCA9685_I2C_ADDRESS_2 PCA9685_I2C_ADDRESS + 1
#define PCA9685_I2C_ADDRESS_3 PCA9685_I2C_ADDRESS + 2
#define NUM_BOARDS 3
#define CHANNELS_PER_BOARD 16
#define NUM_SERVOS NUM_BOARDS * CHANNELS_PER_BOARD

#define SERVO_FREQ_HZ 50 // Standard analog servos

// Pulse length counts (out of 4096) corresponding to servo endpoints.
// These are typical for SG90 / MG90S; tune per servo if needed.
// At 50 Hz, one period = 20 ms = 4096 counts -> 1 count ≈ 4.88 µs
//   500 µs  ≈ 102 counts  (0°)
//   1500 µs ≈ 307 counts  (90°)
//   2500 µs ≈ 512 counts  (180°)
#define SERVO_MIN_COUNT 102
#define SERVO_MAX_COUNT 512


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


void setup() {
  Serial.begin(115200);
  Serial.println(F("Booting 48-servo controller..."));

  Wire.begin();
  Wire.setClock(400000); // 400 kHz fast mode. Might go to 100 kHz

  for (uint8_t i = 0; i < NUM_BOARDS; i++) {
    boards[i].begin();
    boards[i].setOscillatorFrequency(27000000); // Set internal clock to 27 MHz
    boards[i].setPWMFreq(SERVO_FREQ_HZ);
    delay(10);  // Wait for setup so not all start at the same time
  }

  // Soft-start: bring servos to zero one at a time. This avoids a 48-servo simultaneaous inrush burnout.
  Serial.println(F("Starting servos..."));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, 90);
    delay(25); //staggered
  }

}

void loop() {
  //Simple sweep from 0 to 180 deg
  for (uint8_t angle = 0; angle <= 180; angle += 2) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      setServoAngle(i, angle);
    }
    delay(20);
  }

  // Sweep back
  for (int16_t angle = 180; angle >= 0; angle-= 2) {
    for (uint8_t i = 0; i < NUM_SERVOS; i ++) {
      setServoAngle(i, (uint8_t)angle);
    }
    delay(20);
  }

}
