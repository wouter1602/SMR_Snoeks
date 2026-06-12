/*
 * IBT-2 (BTS7960) Gripper Control with Current-Sense Force Limiting
 * ----------------------------------------------------------------
 * Board: Arduino Mega 2560
 *
 * IBT-2 wiring:
 *   VCC  -> 5V
 *   GND  -> GND
 *   RPWM -> PWM pin (forward / close)
 *   LPWM -> PWM pin (reverse / open)
 *   R_EN -> HIGH to enable right half-bridge
 *   L_EN -> HIGH to enable left half-bridge
 *   R_IS -> Analog input (current sense, right side)
 *   L_IS -> Analog input (current sense, left side)
 *
 * The BTS7960 IS pins output a current proportional to load current
 * (ratio ~1:8500). With a pull-down resistor to GND (typ. 1k) you get
 * a voltage you can read with analogRead(). When the gripper grabs an
 * object, current rises sharply -> we stop driving the motor.
 *
 * Serial commands:
 *   o  -> open gripper
 *   c  -> close gripper
 *   s  -> stop
 */

// ---------- Pin assignments ----------
const uint8_t RPWM_PIN = 5;   // PWM, close direction
const uint8_t LPWM_PIN = 6;   // PWM, open direction
const uint8_t R_EN_PIN = 7;
const uint8_t L_EN_PIN = 8;

const uint8_t R_IS_PIN = A1;  // current sense, active when driving RPWM
const uint8_t L_IS_PIN = A2;  // current sense, active when driving LPWM

// ---------- Tuning ----------
const uint8_t  DRIVE_SPEED       = 200;   // 0..255 PWM duty
const int      CURRENT_THRESHOLD = 600;   // ADC counts (0..1023). TUNE THIS!
const uint8_t  OVERCURRENT_HITS  = 3;     // consecutive reads to trigger stop (debounce)
const uint16_t SAMPLE_INTERVAL   = 5;     // ms between current samples
const uint16_t SOFT_START_MS     = 200;   // ignore inrush current at start
const uint16_t MAX_MOVE_TIME_MS  = 2500;  // safety timeout

enum GripperState { IDLE, OPENING, CLOSING };
GripperState state = IDLE;

unsigned long moveStartTime = 0;
unsigned long lastSampleTime = 0;
uint8_t overCurrentCount = 0;

// ---------- Motor control helpers ----------
void openGripper() {
  Serial.println(F("Opening..."));
  state = OPENING;
  moveStartTime = millis();
  lastSampleTime = moveStartTime;
  overCurrentCount = 0;
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, DRIVE_SPEED);
}

void closeGripper() {
  Serial.println(F("Closing..."));
  state = CLOSING;
  moveStartTime = millis();
  lastSampleTime = moveStartTime;
  overCurrentCount = 0;
  analogWrite(LPWM_PIN, 0);
  analogWrite(RPWM_PIN, DRIVE_SPEED);
}

void stopMotor() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  state = IDLE;
  overCurrentCount = 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(R_IS_PIN, INPUT);
  pinMode(L_IS_PIN, INPUT);

  // Enable both half-bridges
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);

  stopMotor();
  Serial.println(F("Gripper ready. Commands: o=open, c=close, s=stop"));
}

void loop() {
  // ---- Handle serial commands ----
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'o': case 'O': openGripper();  break;
      case 'c': case 'C': closeGripper(); break;
      case 's': case 'S': stopMotor();    break;
    }
  }

  // ---- Monitor while moving ----
  if (state != IDLE) {
    unsigned long now = millis();

    // Safety timeout
    if (now - moveStartTime > MAX_MOVE_TIME_MS) {
      Serial.println(F("Timeout reached - stopping."));
      stopMotor();
      return;
    }

    // Sample current at a fixed interval, but skip inrush window
    if ((now - moveStartTime > SOFT_START_MS) &&
        (now - lastSampleTime >= SAMPLE_INTERVAL)) {
      lastSampleTime = now;

      // int sense = (state == CLOSING) ? analogRead(R_IS_PIN)
      //                                : analogRead(L_IS_PIN);

      int sense = 0;

      if (state == CLOSING) {
        sense = analogRead(R_IS_PIN);
      } else {
        sense = analogRead(L_IS_PIN);
      }

      if (sense > CURRENT_THRESHOLD) {
        overCurrentCount++;
        if (overCurrentCount >= OVERCURRENT_HITS) {
          Serial.print(F("Force limit reached (sense="));
          Serial.print(sense);
          Serial.println(F(") - stopping."));
          stopMotor();
        }
      } else {
        overCurrentCount = 0;
      }
    }
  }
}
