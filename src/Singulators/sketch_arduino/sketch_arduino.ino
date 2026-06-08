/*
  Hopper controller protocol

  Commands from transport.py:
    arduino 2 alive
    arduino 2 22-9

  Responses:
    success
    fail

  Set ARDUINO_ID below for each board:
    Board 1 controls hoppers 1-12
    Board 2 controls hoppers 13-24
    Board 3 controls hoppers 25-36

  Each local hopper has:
    - one PWM/output pin that drives the hopper motor/feeder
    - one sensor input pin that pulses once per dropped part
*/

const byte ARDUINO_ID = 2;
const byte HOPPERS_PER_ARDUINO = 12;
const unsigned long COMMAND_TIMEOUT_MS = 30000;
const unsigned long SENSOR_DEBOUNCE_MS = 40;
const byte PWM_SPEED = 180;

// Edit these for your wiring.
const byte hopperOutputPins[HOPPERS_PER_ARDUINO] = {
  2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
};

const byte hopperSensorPins[HOPPERS_PER_ARDUINO] = {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11
};

String inputLine = "";

byte firstGlobalHopper() {
  return ((ARDUINO_ID - 1) * HOPPERS_PER_ARDUINO) + 1;
}

byte lastGlobalHopper() {
  return firstGlobalHopper() + HOPPERS_PER_ARDUINO - 1;
}

int localIndexForHopper(int hopperNumber) {
  if (hopperNumber < firstGlobalHopper() || hopperNumber > lastGlobalHopper()) {
    return -1;
  }
  return hopperNumber - firstGlobalHopper();
}

void stopHopper(int localIndex) {
  analogWrite(hopperOutputPins[localIndex], 0);
}

void startHopper(int localIndex) {
  analogWrite(hopperOutputPins[localIndex], PWM_SPEED);
}

bool dispenseParts(int hopperNumber, int quantity) {
  int localIndex = localIndexForHopper(hopperNumber);
  if (localIndex < 0 || quantity < 1) {
    return false;
  }

  byte sensorPin = hopperSensorPins[localIndex];
  int lastSensorState = digitalRead(sensorPin);
  unsigned long lastPulseAt = 0;
  unsigned long startedAt = millis();
  int counted = 0;

  startHopper(localIndex);

  while (counted < quantity) {
    if (millis() - startedAt > COMMAND_TIMEOUT_MS) {
      stopHopper(localIndex);
      return false;
    }

    int sensorState = digitalRead(sensorPin);
    bool risingEdge = sensorState == HIGH && lastSensorState == LOW;
    bool debounced = millis() - lastPulseAt >= SENSOR_DEBOUNCE_MS;

    if (risingEdge && debounced) {
      counted++;
      lastPulseAt = millis();
    }

    lastSensorState = sensorState;
  }

  stopHopper(localIndex);
  return true;
}

bool parseDispenseCommand(String payload, int &hopperNumber, int &quantity) {
  int dashIndex = payload.indexOf('-');
  if (dashIndex <= 0 || dashIndex >= payload.length() - 1) {
    return false;
  }

  hopperNumber = payload.substring(0, dashIndex).toInt();
  quantity = payload.substring(dashIndex + 1).toInt();
  return hopperNumber > 0 && quantity > 0;
}

void handleCommand(String command) {
  command.trim();
  command.toLowerCase();

  String expectedPrefix = "arduino " + String(ARDUINO_ID) + " ";
  if (!command.startsWith(expectedPrefix)) {
    return;
  }

  String payload = command.substring(expectedPrefix.length());
  payload.trim();

  if (payload == "alive") {
    Serial.println("success");
    return;
  }

  int hopperNumber = 0;
  int quantity = 0;
  if (!parseDispenseCommand(payload, hopperNumber, quantity)) {
    Serial.println("fail");
    return;
  }

  bool ok = dispenseParts(hopperNumber, quantity);
  Serial.println(ok ? "success" : "fail");
}

void setup() {
  Serial.begin(115200);
  inputLine.reserve(48);

  for (byte index = 0; index < HOPPERS_PER_ARDUINO; index++) {
    pinMode(hopperOutputPins[index], OUTPUT);
    analogWrite(hopperOutputPins[index], 0);

    pinMode(hopperSensorPins[index], INPUT_PULLUP);
  }
}

void loop() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    if (incoming == '\n' || incoming == '\r') {
      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += incoming;
    }
  }
}
