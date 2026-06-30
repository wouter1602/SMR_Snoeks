#include "comms.h"
#include "servo_functions.h"
#include "io.h"

uint32_t lastHostMsgMs = 0;
uint32_t lastKeepAliveMs = 0;

uint8_t xorChecksum(const char *s, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) {
    c ^= (uint8_t)s[i]; //Bitwise XOR
  }
  return c;
}

void setupComms() {
  lastHostMsgMs = millis();
  lastKeepAliveMs = millis();
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

bool checkCommsTimeout() {
  return millis() - lastHostMsgMs > HOST_TIMEOUT_MS;
}