#include <Wire.h>
#include "servo_functions.h"
#include "comms.h"
#include "io.h"


void setup() {
  Serial.begin(115200);

  // Serial.println(F("Booting 48-servo controller..."));
  sendLog(("Booting " + String(NUM_SERVOS) + "-servo controller.."));

  Wire.begin();
  Wire.setClock(400000); // 400 kHz fast mode. Might go to 100 kHz

  //Setup IO
  ioSetup();

  //Setup Servos
  setupServos();

  setupComms();

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
  if (checkCommsTimeout()) {
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
