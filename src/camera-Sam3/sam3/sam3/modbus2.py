"""
Conveyor control with Arduino box sensor.

Behaviour:
  - Conveyor starts running forward at SPEED_PCT of MAX_FREQ_HZ.
  - A separate Arduino is connected over USB serial. It continuously prints
    the digital state of a sensor (e.g. an inductive / photoelectric switch)
    as a single character per line: '1' = box detected (HIGH), '0' = clear.
  - When the sensor goes HIGH, the conveyor stops.
  - The operator types  go <Enter>  in the terminal to resume.
  - The conveyor then runs until the *next* rising edge of the sensor
    (a NEW box), where it stops again, and so on.
  - Type  quit <Enter>  (or Ctrl-C) to exit cleanly.

Expected Arduino sketch (minimal):
    const int PIN = 2;
    void setup() { pinMode(PIN, INPUT); Serial.begin(115200); }
    void loop()  { Serial.println(digitalRead(PIN)); delay(20); }
"""

import sys
import time
import threading
import queue
import serial
import minimalmodbus

# ---------------------------------------------------------------------------
# Inverter (FR-D720S) configuration
# ---------------------------------------------------------------------------
STOPBITS      = 1
TIMEOUT_S     = 0.4

MAX_FREQ_HZ   = 120.0
SPEED_PCT     = 3
POLL_PERIOD_S = 0.1

INTER_TX_S    = 0.02
MAX_RETRIES   = 3

# ---------------------------------------------------------------------------
# Arduino configuration
# ---------------------------------------------------------------------------
ARDUINO_PORT     = "/dev/ttyUSB1"
ARDUINO_BAUD     = 115200
ARDUINO_TIMEOUT  = 0.2

# ---------------------------------------------------------------------------
# Inverter (FR-D720S) configuration
# ---------------------------------------------------------------------------
INV_PORT      = "/dev/ttyUSB0"
SLAVE_ADDR    = 1
BAUDRATE      = 19200
PARITY        = serial.PARITY_EVEN
BYTESIZE      = 8
STOPBITS      = 1

# ---------------------------------------------------------------------------
# Register addresses
# ---------------------------------------------------------------------------
REG_CONTROL    = 40009 - 40001
REG_SET_FREQ   = 40014 - 40001
REG_OUT_FREQ   = 40201 - 40001

CTRL_STOP = 0x0001
CTRL_STF  = 0x0002
CTRL_STR  = 0x0004


# ---------------------------------------------------------------------------
# Robust Modbus wrapper
# ---------------------------------------------------------------------------
class CommStats:
    ok = 0
    retried = 0
    failed = 0


def _with_retry(inv, func, *args, **kwargs):
    last_exc = None
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            result = func(*args, **kwargs)
            CommStats.ok += 1
            time.sleep(INTER_TX_S)
            return result
        except (minimalmodbus.NoResponseError,
                minimalmodbus.InvalidResponseError,
                minimalmodbus.LocalEchoError) as e:
            last_exc = e
            CommStats.retried += 1
            try:
                inv.serial.reset_input_buffer()
                inv.serial.reset_output_buffer()
            except Exception:
                pass
            time.sleep(0.05 * attempt)
    CommStats.failed += 1
    raise last_exc


def open_inverter() -> minimalmodbus.Instrument:
    inv = minimalmodbus.Instrument(INV_PORT, SLAVE_ADDR,
                                   mode=minimalmodbus.MODE_RTU)
    inv.serial.baudrate = BAUDRATE
    inv.serial.parity   = PARITY
    inv.serial.bytesize = BYTESIZE
    inv.serial.stopbits = STOPBITS
    inv.serial.timeout  = TIMEOUT_S
    inv.clear_buffers_before_each_transaction = True
    time.sleep(0.2)
    return inv


def write_freq(inv, hz: float) -> None:
    _with_retry(inv, inv.write_register, REG_SET_FREQ,
                int(round(hz * 100)), functioncode=6)


def set_control(inv, bits: int) -> None:
    _with_retry(inv, inv.write_register, REG_CONTROL, bits, functioncode=6)


def read_output_freq(inv) -> float:
    raw = _with_retry(inv, inv.read_register, REG_OUT_FREQ, functioncode=3)
    return raw / 100.0


# ---------------------------------------------------------------------------
# Arduino sensor reader (background thread)
# ---------------------------------------------------------------------------
class SensorReader(threading.Thread):
    def __init__(self, port, baud, timeout):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self.state = 0
        self.rising_edges = queue.Queue()
        self._stop = threading.Event()

    def run(self):
        prev = 0
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
            except Exception:
                continue
            if not line:
                continue
            if line[0] not in ('0', '1'):
                continue
            cur = 1 if line[0] == '1' else 0
            self.state = cur
            if prev == 0 and cur == 1:
                self.rising_edges.put(time.monotonic())
            prev = cur

    def consume_edges(self):
        while not self.rising_edges.empty():
            try:
                self.rising_edges.get_nowait()
            except queue.Empty:
                break

    def wait_for_rising_edge(self) -> bool:
        try:
            self.rising_edges.get_nowait()
            return True
        except queue.Empty:
            return False

    def stop(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Stdin command reader (background thread)
# ---------------------------------------------------------------------------
class CommandReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue()
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                break
            self.q.put(line.strip().lower())

    def get(self):
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None


# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------
def main() -> None:
    print("Opening inverter...")
    inv = open_inverter()
    print(f"  inverter on {INV_PORT}, slave {SLAVE_ADDR}")

    print("Opening Arduino sensor...")
    sensor = SensorReader(ARDUINO_PORT, ARDUINO_BAUD, ARDUINO_TIMEOUT)
    sensor.start()
    print(f"  arduino on {ARDUINO_PORT}")

    cmds = CommandReader()
    cmds.start()

    target_hz = MAX_FREQ_HZ * SPEED_PCT / 100.0
    set_control(inv, CTRL_STOP)
    time.sleep(0.2)
    write_freq(inv, target_hz)

    print("\nCommands:  go = start conveyor   |   quit = exit")
    print(f"Conveyor speed: {SPEED_PCT}% of {MAX_FREQ_HZ} Hz "
          f"= {target_hz:.2f} Hz\n")

    boxes   = 0
    running = False
    armed   = True   # True = ready to detect the next box

    def start_belt():
        nonlocal running, armed
        if running:
            return
        # Clear any edges that fired while we were stopped (current box).
        sensor.consume_edges()
        # If sensor is still HIGH, arm only when it goes LOW again.
        # If sensor is already LOW, arm immediately.
        armed = (sensor.state == 0)
        set_control(inv, CTRL_STF)
        running = True
        print(f"  -> RUNNING (waiting for box #{boxes + 1})")

    def stop_belt(reason=""):
        nonlocal running
        set_control(inv, CTRL_STOP)
        running = False
        if reason:
            print(f"  -> STOPPED ({reason})")

    print("Starting conveyor...")
    start_belt()

    try:
        while True:
            time.sleep(POLL_PERIOD_S)

            # ---- handle keyboard commands ----
            cmd = cmds.get()
            if cmd is not None:
                if cmd in ("q", "quit", "exit"):
                    break
                elif cmd in ("go", "g", "start"):
                    start_belt()
                elif cmd in ("stop", "s"):
                    stop_belt("manual stop")
                elif cmd in ("status", "?"):
                    print(f"  running={running}  sensor={sensor.state}  "
                          f"armed={armed}  boxes={boxes}  "
                          f"comms ok/ret/fail="
                          f"{CommStats.ok}/{CommStats.retried}/"
                          f"{CommStats.failed}")
                else:
                    print(f"  [?] unknown command: {cmd!r} "
                          f"(use: go / stop / status / quit)")

            # ---- react to sensor while running ----
            if running:
                # Re-arm as soon as the current box leaves the sensor
                if sensor.state == 0:
                    armed = True

                # Stop only on a NEW box (rising edge while armed)
                if armed and sensor.wait_for_rising_edge():
                    armed = False
                    boxes += 1
                    try:
                        f_out = read_output_freq(inv)
                    except Exception:
                        f_out = float('nan')
                    print(f"\n[BOX #{boxes}] sensor HIGH "
                          f"at f_out={f_out:.2f} Hz")
                    stop_belt(f"box #{boxes} detected - type 'go' to continue")

    except KeyboardInterrupt:
        print("\n[Ctrl-C] exiting...")
    finally:
        try:
            set_control(inv, CTRL_STOP)
        except Exception:
            pass
        sensor.stop()
        print(f"\nTotal boxes counted: {boxes}")
        print(f"Comms totals: ok={CommStats.ok}  "
              f"retried={CommStats.retried}  failed={CommStats.failed}")


if __name__ == "__main__":
    main()