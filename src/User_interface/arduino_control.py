"""
Simple hopper sequencer.
Just edit main() below: call hopper(id, count) and delay(seconds) in order.
"""

import sys
import time
import serial   # pip install pyserial

# ---------- CONFIG ----------
PORT = "COM4"        # <-- change to your Arduino port (e.g. "/dev/ttyUSB0")
BAUD = 9600
# ----------------------------

ser: serial.Serial = None


def connect():
    global ser
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2.0)   # wait for Arduino reset
    while ser.in_waiting:
        print(ser.readline().decode(errors="ignore").rstrip())


def hopper(hopper_id: int, count: int, timeout: float = 60.0):
    cmd = f"hopper({hopper_id},{count})\n"
    ser.write(cmd.encode())
    ser.flush()
    print(f">> {cmd.strip()}")

    start = time.time()
    while time.time() - start < timeout:
        line = ser.readline().decode(errors="ignore").rstrip()
        if not line:
            continue
        print(f"<< {line}")
        if line == "DONE" or line.startswith("ERR"):
            return
    print("!! timeout")


def delay(seconds: float):
    print(f".. delay {seconds}s")
    time.sleep(seconds)


# ============================================================
# MAIN -- write your sequence here, line by line
# ============================================================
def main():
    connect()

    hopper(1, 3)
    delay(10)
    hopper(2, 2)
    delay(10)
    hopper(1, 3)


    ser.close()


if __name__ == "__main__":
    main()
