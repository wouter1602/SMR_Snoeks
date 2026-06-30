/*
 * Box sensor -> serial bridge for the conveyor controller.
 *
 * Pairs with the Python script (conveyor_box_stop.py) running on the
 * laptop. That script expects ONE character per line, '0' or '1',
 * at 115200 baud:
 *      '1'  = box detected   (sensor HIGH)
 *      '0'  = no box         (sensor LOW)
 *
 * Wiring
 * ------
 *   Sensor signal  -> D2
 *   Sensor GND     -> Arduino GND
 *   Sensor V+      -> 5 V  (or external supply; common GND with Arduino)
 *
 * If your sensor is open-collector / open-drain (typical for cheap
 * inductive / photoelectric NPN sensors), set USE_INTERNAL_PULLUP = true
 * and wire signal -> D2, no external resistor needed. An NPN sensor
 * pulls the line LOW when it detects a target, so we then invert the
 * reading (ACTIVE_LOW = true) so that 'box present' still prints '1'.
 *
 * For a push-pull / PNP sensor that actively drives HIGH on detect,
 * leave USE_INTERNAL_PULLUP = false and ACTIVE_LOW = false.
 *
 * Debounce: a small software debounce filters mechanical / EMI
 * glitches. The line is only re-printed when the *debounced* state
 * actually changes, plus a slow heartbeat so the PC side always sees
 * fresh data even if nothing is happening.
 */

const int  SENSOR_PIN          = 2;
const bool USE_INTERNAL_PULLUP = true;   // true for NPN open-collector
const bool ACTIVE_LOW          = true;   // true for NPN sensors

const unsigned long DEBOUNCE_MS  = 5;    // ignore flips shorter than this
const unsigned long HEARTBEAT_MS = 200;  // resend state at least this often
const unsigned long SAMPLE_US    = 500;  // sampling period (2 kHz)

int           stableState   = 0;   // debounced logical state (1 = box)
int           lastRaw       = 0;   // last raw reading
unsigned long lastFlipMs    = 0;
unsigned long lastPrintMs   = 0;
unsigned long lastSampleUs  = 0;

void setup() {
  if (USE_INTERNAL_PULLUP) {
    pinMode(SENSOR_PIN, INPUT_PULLUP);
  } else {
    pinMode(SENSOR_PIN, INPUT);
  }

  Serial.begin(115200);
  while (!Serial) { /* wait on boards with native USB (Leonardo etc.) */ }

  // Prime the state
  int raw = digitalRead(SENSOR_PIN);
  int logical = ACTIVE_LOW ? (raw == LOW ? 1 : 0)
                           : (raw == HIGH ? 1 : 0);
  stableState  = logical;
  lastRaw      = logical;
  lastFlipMs   = millis();
  lastPrintMs  = millis();
  lastSampleUs = micros();

  Serial.println(stableState);
}

void loop() {
  // Rate-limit sampling so we don't spam the bus with reads.
  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_US) return;
  lastSampleUs = nowUs;

  int raw = digitalRead(SENSOR_PIN);
  int logical = ACTIVE_LOW ? (raw == LOW ? 1 : 0)
                           : (raw == HIGH ? 1 : 0);

  unsigned long nowMs = millis();

  // Debounce: only accept a new stable state if the raw reading has
  // held the new value for at least DEBOUNCE_MS.
  if (logical != lastRaw) {
    lastRaw    = logical;
    lastFlipMs = nowMs;
  } else if (logical != stableState &&
             (nowMs - lastFlipMs) >= DEBOUNCE_MS) {
    stableState = logical;
    Serial.println(stableState);
    lastPrintMs = nowMs;
    return;
  }

  // Heartbeat: keep the PC informed even when nothing changes,
  // so a missed byte doesn't leave it stuck on a stale value.
  if (nowMs - lastPrintMs >= HEARTBEAT_MS) {
    Serial.println(stableState);
    lastPrintMs = nowMs;
  }
}
