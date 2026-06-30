#ifndef COMMS_H
#define COMMS_H
  #include <Arduino.h>

  constexpr uint32_t HOST_TIMEOUT_MS = 30000; // No traffic --> asume disconnected
  constexpr uint32_t KEEPALIVE_INTERVAL_MS = 1000; // Send "KA" this often

  void setupComms();


  void sendLine(const String &payload);
  void sendLog(const String &msg);
  void sendProgress(uint32_t seq, uint8_t servo, int completed, int total);
  void maybeSendKeepAlive();

  String validateAndStrip(const String &line);
  void handleCommand(const String &raw);
  void pollSerial();
  void busyDelay(uint32_t ms);

  bool performOnCycle(uint8_t servo);
  bool checkCommsTimeout();


#endif //COMMS_H