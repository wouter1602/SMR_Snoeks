
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


constexpr uint8_t PCA9685_I2C_ADDRESS_1  = PCA9685_I2C_ADDRESS;
constexpr uint8_t PCA9685_I2C_ADDRESS_2 = PCA9685_I2C_ADDRESS + 1;
constexpr uint8_t PCA9685_I2C_ADDRESS_3 = PCA9685_I2C_ADDRESS + 2;
constexpr uint8_t NUM_BOARDS = 3;
constexpr uint8_t CHANNELS_PER_BOARD = 16;
constexpr uint8_t NUM_SERVOS = NUM_BOARDS * CHANNELS_PER_BOARD;

constexpr uint8_t SERVO_FREQ_HZ = 50; //Standard frequency for analog servos
constexpr uint8_t MAX_RETRIES = 3;
constexpr uint32_t HOST_TIMEOUT_MS = 30000; // No traffic --> asume disconnected
constexpr uint32_t KEEPALIVE_INTERVAL_MS = 1000; // Send "KA" this often


constexpr uint16_t MOVE_SETTLE_MS = 250; // Wait after commanding move

// Pulse length counts (out of 4096) corresponding to servo endpoints.
// These are typical for SG90 / MG90S; tune per servo if needed.
// At 50 Hz, one period = 20 ms = 4096 counts -> 1 count ≈ 4.88 µs
//   500 µs  ≈ 102 counts  (0°)
//   1500 µs ≈ 307 counts  (90°)
//   2500 µs ≈ 512 counts  (180°)
constexpr uint16_t SERVO_MIN_COUNT = 102;
constexpr uint16_t SERVO_MAX_COUNT = 512;
constexpr uint8_t SERVO_ACTIVE_ANGLE = 90; //Need some fine tuneing
constexpr uint8_t SERVO_REST_ANGLE = 0; //Need some finetuneing.

uint32_t lastHostMsgMs = 0;
uint32_t lastKeepAliveMs = 0;


Adafruit_PWMServoDriver boards[NUM_BOARDS] = {
  Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS_1),
  Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS_2),
  Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS_3)
};

constexpr uint8_t HopperSensorPins[NUM_SERVOS] = {
  12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 22, 24, 25, 26, 27, 28, 29,
  30, 32, 32, 33, 34, 35, 37, 38, 44, 43, 42, 41, 40, 29, 53, 52,
  51, 50, 49, 48, 47, 46, 45
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

// Convert microseconds to PCA9685 12-bit count at SERVO_FREQ_HZ
// TODO: is this needed?
uint16_t usToTICKS(uint16_t us) {
  // Period in us = 1e6 / freq
  uint32_t period_us = 1000000UL / SERVO_FREQ_HZ;
  return (uint32_t)us * 4096UL / period_us;
}

void setServoUs(uint8_t servo, uint16_t us) {
  uint16_t ticks = usToTICKS(us);
  // TODO: Based on servo go to correct module.
}

bool sensorActive(uint8_t servo) {
  return digitalRead(HopperSensorPins[servo]) == HIGH; // 1 = detected
}

// Non-blocking-ish delay that keeps the serial layer alive (keep-alives,
// incoming PINGs, etc.) while we wait for a servo to settle.
void busyDelay(uint32_t ms);

// Perform a single "actuate + return" cycle with verificaitn.
// Returns true on success within MAX_RETRIES.
bool performOnCycle(uint8_t servo) {
  for (uint8_t attempt = 0; attempt < MAX_RETRIES; attempt++) {
    // 1. Move to actuated position
    setServoAngle(servo, SERVO_ACTIVE_ANGLE);
    busyDelay(MOVE_SETTLE_MS);
    if (!sensorActive(servo)) { //Propably wont work
      sendLog("Servo " + String(servo) + " no trigger, attempt " + String(attempt+1));
      //retry: bounce back briefly
      setServoAngle(servo, SERVO_ACTIVE_ANGLE);
      busyDelay(MOVE_SETTLE_MS);
      continue;
    }

    // 2. Return to rest
    setServoAngle(servo, SERVO_REST_ANGLE);
    if (sensorActive(servo)) {
      sendLog("Servo " + String(servo) + " stuck triggered, attempt " + String(attempt + 1));
      continue;
    }
    return true;
  }
  return false;
}

/**********************************************
 *                                            *
 *             Communication commands         *
 *                                            *
 **********************************************/

uint8_t xorChecksum(const char *s, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) {
    c ^= (uint8_t)s[i]; //Bitwise XOR
  }
  return c;
}

void sendLine(const String &payload) {
  uint8_t c = xorChecksum(payload.c_str(), payload.length());
  char buf[8];
  snprintf(buf, sizeof(buf), "*%02x", c);
  Serial.print(payload);
  Serial.println(buf);
}

void sendLog(const String &msg) {
  sendLine("LOG| " + msg);
}

void sendProgress(uint32_t seq, uint8_t servo, int completed, int total) {
  sendLine(String(seq) + "|PROG|" + String(servo) + "|" + String(completed) + "|" + String(total));
}

// Periodic keep-alive so the host knows we're still here, even when we are
// busy in the middle of a long MOVE command and not sending PROG often
void maybeSendKeepAlive() {
  uint32_t now = millis();
  if (now - lastKeepAliveMs >= KEEPALIVE_INTERVAL_MS) {
    lastKeepAliveMs = now;
    sendLine("KA");
  }
}

/******************
 *                *
 * COMMAND PARSER *
 *                *
 ******************/
// Validate "<payload>*<xor>" and return payload stirng (without checksum).
// Returs empty String on failure.
String validateAndStrip(const String &line) {
  int star = line.lastIndexOf('*');
  if (star < 0 || star + 3 > (int)line.length()) { //Return empy string if star is 0 or more than 3.
    return String();
  }
  String payload = line.substring(0, star);
  String hex = line.substring(star + 1);
  hex.trim();
  uint8_t got = (uint8_t)strtoul(hex.c_str(), nullptr, 16);
  uint8_t calc = xorChecksum(payload.c_str(), payload.length());
  if (got != calc) { // Return empty string if calc and got are not the same (XOR failed)
    return String();
  }
  return payload;
}

void handleCommand(const String &raw) {
  String payload = validateAndStrip(raw);
  if (payload.length() == 0) {
    sendLog("bad checksum");
    return;
  }
  lastHostMsgMs = millis();

  if (payload == "PING") { sendLine("PONG"); return; }
  if (payload == "PONG") { return; }
  if (payload == "KA")   { return; } // host keep-alive

  // tokenize by '|'
  String tokens[5];
  int n = 0;
  int start = 0;
  for (int i = 0; i <= (int)payload.length() && n < 5; i++) {
    if (i == (int)payload.length() || payload[i] == '|') {
      tokens[n++] = payload.substring(start, i);
      start = i + 1;
    }
  }

  if (n >= 4 && tokens[1] == "MOVE") {
    uint32_t seq = tokens[0].toInt();
    int servo = tokens[2].toInt();
    int count = tokens[3].toInt();

    if (servo < 0 || servo >= NUM_SERVOS || count <= 0 ) {
      sendLine(String(seq) + "|RES|" + String(servo) + "|0|Fail");
      return;
    }

    int completed = 0;
    bool ok = true;
    for (int i = 0; i < count; i++) {
      if (performOnCycle((uint8_t)servo)) {
        completed++;
        // Push an intermediat status update for every successful cycle.
        sendProgress(seq, (uint8_t)servo, completed, count);
      } else {
        ok = false;
        break;
      }
      maybeSendKeepAlive();
    }
    String status = ok ? "OK" : "FAIL";
    sendLine(String(seq) + "|RES|" + String(servo) + "|" + String(completed) + "|" + status);
    return;
  }

  sendLog("unknown cmd: " + payload);
}

// Serial line reader
String rxBuf;
void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (rxBuf.length()) handleCommand(rxBuf);
      rxBuf = "";
    } else {
      if (rxBuf.length() < 128) rxBuf += c;
      else rxBuf = ""; // overflow -> drop
    }
  }
}

// Implementation placed after pollSerial so it can call it.
void busyDelay(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    pollSerial();
    maybeSendKeepAlive();
    delay(1);
  }
}


void setup() {
  Serial.begin(115200);

  // Serial.println(F("Booting 48-servo controller..."));
  sendLog(("Booting " + String(NUM_SERVOS) + "-servo controller.."));

  Wire.begin();
  Wire.setClock(400000); // 400 kHz fast mode. Might go to 100 kHz

  for (uint8_t i = 0; i < NUM_BOARDS; i++) {
    pinMode(HopperSensorPins[i], INPUT);
  }

  //Setup Servos
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

  lastHostMsgMs = millis();
  lastKeepAliveMs = millis();

  sendLog("Ready");

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

  pollSerial();
  maybeSendKeepAlive();

  // if host went silent, park everything safely.
  static bool parked = false;
  if (millis() - lastHostMsgMs > HOST_TIMEOUT_MS) {
    if (!parked) {
      for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        setServoAngle(i, SERVO_REST_ANGLE);
        parked = true;
        sendLog("host timeout - parked");
      }
    } else {
      parked = false;
    }
  }

}
