/***************************************************************
 * combined_controller_v3.ino
 * --------------------------------------------------------------
 * Eén Arduino-sketch die TWEE rollen tegelijk vervult:
 *
 * 1) BOX-SENSOREN voor de transportband
 *    - 4 digitale sensoren op D2, D4, D5, D6
 *      (D2 = row 4 / S0, D4 = row 3 / S1,
 *       D5 = row 2 / S2, D6 = row 1 / S3)
 *    - Sampling @ ~50 Hz met debounce
 *    - Print elk sensor-event op een eigen regel:
 *           S0:1   S0:0   S1:1   ...
 *      Alleen bij verandering (edge-triggered), plus
 *      1x per sensor bij opstart.
 *
 * 2) HOPPER-CONTROLLER (25 hoppers via 3x PCA9685) + Y-as servo
 *    - Y-as servo op pin D3
 *    - Seriële commando's:
 *         hopper(<id>,<count>)\n   -> dispense
 *         yas(<a|b|c>)\n           -> verplaats Y-as
 *         stop\n                   -> abort huidig dispense
 *    - Antwoorden: DONE / SKIPPED / ERR: ...
 *
 * Beide rollen delen dezelfde USB-serial @ 115200 baud.
 ***************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

// ============================================================
//  BOX SENSORS (conveyor) — 4 stuks
// ============================================================
const uint8_t  NUM_BOX_SENSORS        = 4;
const uint8_t  BOX_SENSOR_PINS[NUM_BOX_SENSORS] = { 5, 4, 2, 6 };
const bool     BOX_SENSOR_USE_PULLUP  = true;
const bool     BOX_SENSOR_INVERT      = true;   // active-low NPN
const uint16_t BOX_SENSOR_PERIOD_MS   = 20;     // ~50 Hz
const uint8_t  BOX_SENSOR_DEBOUNCE_N  = 2;
const uint8_t  LED_PIN                = LED_BUILTIN;

uint8_t  bsStable      [NUM_BOX_SENSORS];
uint8_t  bsCandidate   [NUM_BOX_SENSORS];
uint8_t  bsRuns        [NUM_BOX_SENSORS];
uint8_t  bsLastPrinted [NUM_BOX_SENSORS];
unsigned long bsNextMs = 0;

// ============================================================
//  PWM boards (hoppers)
// ============================================================
#define NUM_BOARDS 3
Adafruit_PWMServoDriver pwm[NUM_BOARDS] = {
  Adafruit_PWMServoDriver(0x40),
  Adafruit_PWMServoDriver(0x41),
  Adafruit_PWMServoDriver(0x42),
};
#define SERVO_FREQ 50
#define USMIN      600
#define USMAX      2400

// ============================================================
//  Y-axis servo (D3)
// ============================================================
#define Y_SERVO_PIN 3
Servo yServo;

const int Y_ANGLE_A = 0;
const int Y_ANGLE_B = 90;
const int Y_ANGLE_C = 180;

int        yCurrentAngle    = -1;
const uint8_t Y_STEP_DELAY_MS = 15;

// ============================================================
//  Hopper config
// ============================================================
struct Hopper {
  uint8_t  board;
  uint8_t  dispenseCh;
  uint8_t  gateCh;
  uint8_t  sensorPin;
  int      restAngle;
  int      dispenseAngle;
  int      gateOpenAngle;
  int      gateClosedAngle;
  uint8_t  stepDelayMs;
  uint16_t pauseMs;
};

Hopper hoppers[] = {
  /* id  1  S-1 */ { 0,  0,  1, 22, 90, 90, 0, 90, 25, 400 },
  /* id  2  S-2 */ { 0,  2,  3, 23, 90, 90, 0, 90, 25, 400 },
  /* id  3  S-3 */ { 0,  4,  5, 24, 90, 90, 0, 90, 25, 400 },
  /* id  4  S-4 */ { 0,  6,  7, 25, 90, 90, 0, 90, 25, 400 },
  /* id  5  S-5 */ { 0,  8,  9, 26, 90, 90, 0, 90, 25, 400 },
  /* id  6  M-1 */ { 0, 10, 11, 27, 90, 75, 0, 90, 30, 400 },
  /* id  7  M-2 */ { 0, 12, 13, 28, 90, 75, 0, 90, 30, 400 },
  /* id  8  M-3 */ { 0, 14, 15, 29, 90, 75, 0, 90, 30, 400 },
  /* id  9  M-4 */ { 1,  0,  1, 30, 90, 75, 0, 90, 30, 400 },
  /* id 10  M-5 */ { 1,  2,  3, 31, 90, 75, 0, 90, 30, 400 },
  /* id 11  L-1 */ { 1,  4,  5, 32, 90, 60, 0, 90, 20, 400 },
  /* id 12  L-2 */ { 1,  6,  7, 33, 90, 60, 0, 90, 20, 400 },
  /* id 13  L-3 */ { 1,  8,  9, 34, 90, 60, 0, 90, 20, 400 },
  /* id 14  L-4 */ { 1, 10, 11, 35, 90, 60, 0, 90, 20, 400 },
  /* id 15  L-5 */ { 1, 12, 13, 36, 90, 60, 0, 90, 20, 400 },
  /* id 16  W-1 */ { 1, 14, 15, 37, 90, 45, 0, 90, 35, 500 },
  /* id 17  W-2 */ { 2,  0,  1, 38, 90, 45, 0, 90, 35, 500 },
  /* id 18  W-3 */ { 2,  2,  3, 39, 90, 45, 0, 90, 35, 500 },
  /* id 19  W-4 */ { 2,  4,  5, 40, 90, 45, 0, 90, 35, 500 },
  /* id 20  W-5 */ { 2,  6,  7, 41, 90, 45, 0, 90, 35, 500 },
  /* id 21  W-6 */ { 2,  8,  9, 42, 90, 45, 0, 90, 35, 500 },
  /* id 22  W-7 */ { 2, 10, 11, 43, 90, 45, 0, 90, 35, 500 },
  /* id 23  W-8 */ { 2, 12, 13, 47, 90, 45, 0, 90, 35, 500 },
  /* id 24  W-9 */ { 2, 14, 15, 48, 90, 45, 0, 90, 35, 500 },
  /* id 25  W-10*/ { 2,  0,  1, 49, 90, 45, 0, 90, 35, 500 },
};
const uint8_t NUM_HOPPERS = sizeof(hoppers) / sizeof(hoppers[0]);

// ============================================================
//  Gate state
// ============================================================
const unsigned long GATE_DEBOUNCE_MS = 3000;
bool          gateClosed       [NUM_HOPPERS];
unsigned long sensorStateSince [NUM_HOPPERS];
bool          lastSensorState  [NUM_HOPPERS];

// ============================================================
//  Serial / abort
// ============================================================
String rxBuf = "";
bool   stopRequested = false;

// ============================================================
//  BOX SENSOR SAMPLING (non-blocking, edge-triggered output)
// ============================================================
static inline void printSensor(uint8_t i, uint8_t v) {
  Serial.print('S'); Serial.print(i); Serial.print(':'); Serial.println(v ? '1' : '0');
}

void sampleBoxSensors() {
  unsigned long now = millis();
  if ((long)(now - bsNextMs) < 0) return;
  bsNextMs = now + BOX_SENSOR_PERIOD_MS;

  for (uint8_t i = 0; i < NUM_BOX_SENSORS; i++) {
    uint8_t raw = digitalRead(BOX_SENSOR_PINS[i]);
    if (BOX_SENSOR_INVERT) raw = !raw;

    if (raw == bsCandidate[i]) {
      if (bsRuns[i] < 255) bsRuns[i]++;
    } else {
      bsCandidate[i] = raw;
      bsRuns[i] = 1;
    }
    if (bsRuns[i] >= BOX_SENSOR_DEBOUNCE_N && bsCandidate[i] != bsStable[i]) {
      bsStable[i] = bsCandidate[i];
    }
    if (bsStable[i] != bsLastPrinted[i]) {
      printSensor(i, bsStable[i]);
      bsLastPrinted[i] = bsStable[i];
    }
  }

  // LED weerspiegelt sensor 0 (row 4)
  digitalWrite(LED_PIN, bsStable[0] ? HIGH : LOW);
}

// ============================================================
//  Helpers
// ============================================================
void setServoAngle(uint8_t board, uint8_t ch, int angle) {
  if (board >= NUM_BOARDS) return;
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;
  int us = map(angle, 0, 180, USMIN, USMAX);
  pwm[board].writeMicroseconds(ch, us);
}

bool checkStop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      String tmp = rxBuf;
      tmp.trim();
      tmp.toLowerCase();
      rxBuf = "";
      if (tmp == "stop") stopRequested = true;
    } else {
      rxBuf += c;
      if (rxBuf.length() > 64) rxBuf = "";
    }
  }
  if (stopRequested) {
    stopRequested = false;
    return true;
  }
  return false;
}

void updateGates() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_HOPPERS; i++) {
    bool s = digitalRead(hoppers[i].sensorPin) == HIGH;
    if (s != lastSensorState[i]) {
      lastSensorState[i] = s;
      sensorStateSince[i] = now;
    }
    if ((now - sensorStateSince[i]) >= GATE_DEBOUNCE_MS) {
      if (s && !gateClosed[i]) {
        setServoAngle(hoppers[i].board, hoppers[i].gateCh, hoppers[i].gateClosedAngle);
        gateClosed[i] = true;
        Serial.print(F("Gate CLOSED hopper ")); Serial.println(i + 1);
      } else if (!s && gateClosed[i]) {
        setServoAngle(hoppers[i].board, hoppers[i].gateCh, hoppers[i].gateOpenAngle);
        gateClosed[i] = false;
        Serial.print(F("Gate OPEN hopper "));   Serial.println(i + 1);
      }
    }
  }
}

inline void tick() {
  sampleBoxSensors();
  updateGates();
}

bool sweepServo(uint8_t board, uint8_t ch, int from, int to, uint8_t stepDelayMs) {
  int step = (to >= from) ? 1 : -1;
  for (int a = from; a != to; a += step) {
    setServoAngle(board, ch, a);
    delay(stepDelayMs);
    tick();
    if (checkStop()) {
      setServoAngle(board, ch, to);
      return true;
    }
  }
  setServoAngle(board, ch, to);
  return false;
}

bool abortableDelay(uint16_t ms) {
  unsigned long end = millis() + ms;
  while ((long)(end - millis()) > 0) {
    delay(5);
    tick();
    if (checkStop()) return true;
  }
  return false;
}

// ============================================================
//  Y-axis
// ============================================================
void moveYAs(char x) {
  int target;
  switch (x) {
    case 'a': case 'A': target = Y_ANGLE_A; break;
    case 'b': case 'B': target = Y_ANGLE_B; break;
    case 'c': case 'C': target = Y_ANGLE_C; break;
    default:
      Serial.print(F("ERR: bad y-as letter: ")); Serial.println(x);
      return;
  }

  Serial.print(F("Y-as -> ")); Serial.print((char)toupper(x));
  Serial.print(F(" (")); Serial.print(target); Serial.println(F(" deg)"));

  if (yCurrentAngle < 0) {
    yServo.write(target);
  } else {
    int step = (target >= yCurrentAngle) ? 1 : -1;
    for (int a = yCurrentAngle; a != target; a += step) {
      yServo.write(a);
      delay(Y_STEP_DELAY_MS);
      tick();
    }
    yServo.write(target);
  }
  yCurrentAngle = target;
  Serial.println(F("DONE"));
}

// ============================================================
//  Dispense
// ============================================================
void dispense(uint8_t hopperIdx, uint16_t count) {
  if (hopperIdx >= NUM_HOPPERS) {
    Serial.print(F("ERR: invalid hopper ")); Serial.println(hopperIdx + 1);
    return;
  }
  Hopper &h = hoppers[hopperIdx];
  Serial.print(F("Dispensing ")); Serial.print(count);
  Serial.print(F(" from hopper ")); Serial.println(hopperIdx + 1);

  int flickAngle = h.restAngle - h.dispenseAngle;

  for (uint16_t i = 0; i < count; i++) {
    if (sweepServo(h.board, h.dispenseCh, h.restAngle, flickAngle, h.stepDelayMs)) {
      Serial.println(F("SKIPPED")); return;
    }
    if (abortableDelay(h.pauseMs)) { Serial.println(F("SKIPPED")); return; }

    if (sweepServo(h.board, h.dispenseCh, flickAngle, h.restAngle, h.stepDelayMs)) {
      Serial.println(F("SKIPPED")); return;
    }
    if (abortableDelay(h.pauseMs)) { Serial.println(F("SKIPPED")); return; }
  }
  Serial.println(F("DONE"));
}

// ============================================================
//  Command parser
// ============================================================
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) return;

  if (cmd == "stop") {
    Serial.println(F("SKIPPED"));
    return;
  }

  if (cmd.startsWith("yas(") && cmd.endsWith(")")) {
    String inside = cmd.substring(4, cmd.length() - 1);
    inside.trim();
    if (inside.length() != 1) {
      Serial.println(F("ERR: yas expects 1 letter"));
      return;
    }
    moveYAs(inside.charAt(0));
    return;
  }

  if (!cmd.startsWith("hopper(") || !cmd.endsWith(")")) {
    Serial.print(F("ERR: bad cmd: ")); Serial.println(cmd);
    return;
  }
  int comma = cmd.indexOf(',');
  if (comma < 0) { Serial.println(F("ERR: missing comma")); return; }

  int id    = cmd.substring(7, comma).toInt();
  int count = cmd.substring(comma + 1, cmd.length() - 1).toInt();
  if (id < 1 || count < 1) { Serial.println(F("ERR: bad args")); return; }

  dispense(id - 1, count);
}

// ============================================================
//  setup / loop
// ============================================================
void setup() {
  // Box-sensor pins
  for (uint8_t i = 0; i < NUM_BOX_SENSORS; i++) {
    pinMode(BOX_SENSOR_PINS[i], BOX_SENSOR_USE_PULLUP ? INPUT_PULLUP : INPUT);
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { /* spin */ }

  Serial.println(F("Combined controller v3 ready (4 box sensors + 25 hoppers + Y-as)"));

  // PWM boards
  for (uint8_t b = 0; b < NUM_BOARDS; b++) {
    pwm[b].begin();
    pwm[b].setOscillatorFrequency(27000000);
    pwm[b].setPWMFreq(SERVO_FREQ);
  }
  delay(10);

  // Y-as servo init -> positie A
  yServo.attach(Y_SERVO_PIN);
  yServo.write(Y_ANGLE_A);
  yCurrentAngle = Y_ANGLE_A;
  Serial.println(F("Y-as init -> A (pin D3)"));

  // Hoppers init
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_HOPPERS; i++) {
    pinMode(hoppers[i].sensorPin, INPUT);
    setServoAngle(hoppers[i].board, hoppers[i].dispenseCh, hoppers[i].restAngle);
    setServoAngle(hoppers[i].board, hoppers[i].gateCh, 0);

    gateClosed[i]       = false;
    lastSensorState[i]  = digitalRead(hoppers[i].sensorPin) == HIGH;
    sensorStateSince[i] = now;
  }
  Serial.println(F("All gates forced to 0 deg"));

  // Init box-sensor stable state + 1x rapporteren per sensor bij opstart
  for (uint8_t i = 0; i < NUM_BOX_SENSORS; i++) {
    uint8_t raw = digitalRead(BOX_SENSOR_PINS[i]);
    if (BOX_SENSOR_INVERT) raw = !raw;
    bsStable[i]      = raw;
    bsCandidate[i]   = raw;
    bsRuns[i]        = BOX_SENSOR_DEBOUNCE_N;
    bsLastPrinted[i] = raw;
    printSensor(i, raw);
  }
  bsNextMs = millis();
}

void loop() {
  // Achtergrond: box sensoren streamen + hopper gates onderhouden
  tick();

  // Seriële command parsing
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxBuf.length() > 0) {
        String cmd = rxBuf;
        rxBuf = "";
        handleCommand(cmd);
      }
    } else {
      rxBuf += c;
      if (rxBuf.length() > 64) rxBuf = "";
    }
  }
}
