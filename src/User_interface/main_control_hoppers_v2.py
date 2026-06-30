"""
Hopper sequencer v2
-------------------
Wijziging t.o.v. v1:
- Er is fysiek maar één box-sensor (S0) op de Arduino. De overgangen
  tussen rij 4 -> 3 -> 2 -> 1 worden daarom NIET meer op sensor S1/S2/S3
  gedaan, maar op TIJD (ROW_ADVANCE_SECONDS). Pas die waarde aan
  zodat de doos precies één rij doorschuift.
- Finale eject blijft een vaste tijd lopen.
- Sensorregels van de Arduino in formaat "S0:<0|1>" worden nog steeds
  netjes gefilterd zodat ze niet als hopper-antwoord worden gezien.
"""

import json
import time
from pathlib import Path
import serial  # pip install pyserial

# ---------- CONFIG ----------
ARDUINO_PORT      = "COM7"
ARDUINO_BAUD      = 115200

INV_PORT          = "COM5"
INV_SLAVE_ADDR    = 1
INV_BAUDRATE      = 19200
INV_BYTESIZE      = 8
INV_STOPBITS      = 1
INV_TIMEOUT_S     = 0.4
MAX_FREQ_HZ       = 120.0
SPEED_PCT         = 3
INTER_TX_S        = 0.02
MODBUS_RETRIES    = 3

REG_CONTROL       = 40009 - 40001
REG_SET_FREQ      = 40014 - 40001
CTRL_STOP         = 0x0001
CTRL_STF          = 0x0002

BATCH_FILE        = Path(__file__).parent / "data" / "active_batch.json"
DELAY_BETWEEN     = 5.0
HOPPER_TIMEOUT    = 600.0
ABORT_WAIT        = 2.0
YAS_TIMEOUT       = 10.0

# Tijd dat de band moet draaien om de doos exact één rij door te
# schuiven. KALIBREREN aan de hand van je rij-afstand en bandsnelheid.
ROW_ADVANCE_SECONDS = 3.0
FINAL_EJECT_SECONDS = 5.0
# ----------------------------

ser: serial.Serial = None
current_letter: str = None
current_row: int = None

# Sensor state (alleen S0 wordt nu nog gebruikt, maar we houden de dict
# generiek voor het geval er later sensoren bijkomen).
sensor_state = {0: 0}
sensor_rising_pending = {0: False}


# ---------- KEYBOARD ----------
try:
    import msvcrt
    def kb_hit(): return msvcrt.kbhit()
    def kb_get():
        ch = msvcrt.getch()
        try: return ch.decode(errors="ignore").lower()
        except Exception: return ""
except ImportError:
    def kb_hit(): return False
    def kb_get(): return ""


def check_keys():
    key = None
    while kb_hit():
        k = kb_get()
        if k in ("s", "q"):
            key = k
    return key


# ---------- SERIAL ----------
def connect():
    global ser
    ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.05)
    time.sleep(2.0)
    while ser.in_waiting:
        raw = ser.readline().decode(errors="ignore").rstrip()
        _ingest_line(raw)
        if raw:
            print(raw)


def _ingest_line(line: str):
    """
    Sensorregels "S<idx>:<0|1>" -> sensor_state bijwerken, return None.
    Andere regels -> ongewijzigd terug.
    """
    if not line:
        return None
    if len(line) >= 4 and line[0] == "S" and line[1].isdigit() and line[2] == ":":
        try:
            idx = int(line[1])
            val = 1 if line[3] == "1" else 0
        except Exception:
            return line
        if idx in sensor_state:
            prev = sensor_state[idx]
            sensor_state[idx] = val
            if prev == 0 and val == 1:
                sensor_rising_pending[idx] = True
        return None
    return line


def read_arduino_line():
    line = ser.readline().decode(errors="ignore").rstrip()
    out = _ingest_line(line)
    return out if out else ""


def send_stop_and_wait():
    try:
        ser.write(b"stop\n"); ser.flush()
    except Exception as e:
        print(f"!! could not send stop: {e}")
        return
    end = time.time() + ABORT_WAIT
    while time.time() < end:
        line = read_arduino_line()
        if not line: continue
        print(f"<< {line}")
        if line in ("SKIPPED", "DONE") or line.startswith("ERR"):
            return


def move_y_as(letter: str):
    global current_letter
    letter = letter.lower()
    if letter not in ("a", "b", "c"):
        print(f"!! ongeldige Y-as letter: {letter}")
        return
    if letter == current_letter:
        return

    cmd = f"yas({letter})\n"
    ser.write(cmd.encode()); ser.flush()
    print(f">> {cmd.strip()}")

    start = time.time()
    while time.time() - start < YAS_TIMEOUT:
        line = read_arduino_line()
        if not line: continue
        print(f"<< {line}")
        if line == "DONE":
            current_letter = letter
            return
        if line.startswith("ERR"):
            return
    print("!! Y-as timeout")


def hopper(hopper_id: int, count: int, timeout: float = HOPPER_TIMEOUT):
    cmd = f"hopper({hopper_id},{count})\n"
    ser.write(cmd.encode()); ser.flush()
    print(f">> {cmd.strip()}   (press 's'=skip, 'q'=quit)")

    start = time.time()
    while time.time() - start < timeout:
        key = check_keys()
        if key == "s":
            print("!! skip pressed -- sending stop")
            send_stop_and_wait()
            return "skip"
        if key == "q":
            print("!! quit pressed -- sending stop")
            send_stop_and_wait()
            return "quit"
        line = read_arduino_line()
        if not line: continue
        print(f"<< {line}")
        if line == "DONE":    return "done"
        if line == "SKIPPED": return "skip"
        if line.startswith("ERR"): return "err"

    print("!! timeout -- sending stop")
    send_stop_and_wait()
    return "timeout"


def delay(seconds: float):
    print(f".. delay {seconds}s (press 's' to skip, 'q' to quit)")
    end = time.time() + seconds
    while time.time() < end:
        key = check_keys()
        if key == "s":
            print(".. delay skipped"); return "skip"
        if key == "q":
            return "quit"
        line = read_arduino_line()
        if line:
            print(f"<< {line}")
    return "done"


# ============================================================
#  CONVEYOR (Modbus inverter op COM5)
# ============================================================
class Conveyor:
    def __init__(self):
        import serial as pyserial
        import minimalmodbus
        self._mm = minimalmodbus
        self.inv = minimalmodbus.Instrument(
            INV_PORT, INV_SLAVE_ADDR, mode=minimalmodbus.MODE_RTU
        )
        self.inv.serial.baudrate = INV_BAUDRATE
        self.inv.serial.parity   = pyserial.PARITY_EVEN
        self.inv.serial.bytesize = INV_BYTESIZE
        self.inv.serial.stopbits = INV_STOPBITS
        self.inv.serial.timeout  = INV_TIMEOUT_S
        self.inv.clear_buffers_before_each_transaction = True
        time.sleep(0.2)
        self.running = False
        self._configure_speed()

    def _with_retry(self, func, *args, **kwargs):
        mm = self._mm
        last = None
        for attempt in range(1, MODBUS_RETRIES + 1):
            try:
                result = func(*args, **kwargs)
                time.sleep(INTER_TX_S)
                return result
            except (mm.NoResponseError,
                    mm.InvalidResponseError,
                    mm.LocalEchoError) as e:
                last = e
                try:
                    self.inv.serial.reset_input_buffer()
                    self.inv.serial.reset_output_buffer()
                except Exception:
                    pass
                time.sleep(0.05 * attempt)
        raise last

    def _configure_speed(self):
        target_hz = MAX_FREQ_HZ * SPEED_PCT / 100.0
        self._with_retry(self.inv.write_register,
                         REG_CONTROL, CTRL_STOP, functioncode=6)
        time.sleep(0.2)
        self._with_retry(self.inv.write_register,
                         REG_SET_FREQ, int(round(target_hz * 100)),
                         functioncode=6)
        print(f"[CONVEYOR] Snelheid ingesteld op {target_hz:.2f} Hz")

    def start(self):
        if self.running: return
        self._with_retry(self.inv.write_register,
                         REG_CONTROL, CTRL_STF, functioncode=6)
        self.running = True
        print("[CONVEYOR] START")

    def stop(self):
        try:
            self._with_retry(self.inv.write_register,
                             REG_CONTROL, CTRL_STOP, functioncode=6)
        except Exception as e:
            print(f"[CONVEYOR] !! kon niet stoppen: {e}")
        self.running = False
        print("[CONVEYOR] STOP")

    def close(self):
        try: self.stop()
        except Exception: pass
        try: self.inv.serial.close()
        except Exception: pass


def conveyor_run_for(conv: Conveyor, seconds: float, label: str = ""):
    """Band X seconden laten lopen, serial tegelijk leegtrekken."""
    if label:
        print(f"[CONVEYOR] {label} ({seconds:.1f}s)")
    else:
        print(f"[CONVEYOR] Run {seconds:.1f}s")
    conv.start()
    end = time.time() + seconds
    while time.time() < end:
        line = read_arduino_line()
        if line:
            print(f"<< {line}")
        key = check_keys()
        if key == "q":
            conv.stop()
            return False
        time.sleep(0.02)
    conv.stop()
    return True


# ============================================================
#  SEQUENCE
# ============================================================
HOPPER_COUNTS      = {"S": 5, "M": 5, "L": 5, "W": 10}
HOPPER_PREFIX_BASE = {}
_offset = 0
for _prefix in ("S", "M", "L", "W"):
    HOPPER_PREFIX_BASE[_prefix] = _offset
    _offset += HOPPER_COUNTS[_prefix]

BUFFER_ORDER = [
    "A4", "B4", "C4",
    "A3", "B3", "C3",
    "A2", "B2", "C2",
    "A1", "B1", "C1",
]


def hopper_name_to_id(name: str) -> int:
    prefix, num = name.split("-")
    num = int(num)
    if num < 1 or num > HOPPER_COUNTS[prefix]:
        raise ValueError(f"Hopper {name} out of range")
    return HOPPER_PREFIX_BASE[prefix] + num


def build_sequence(batch_path):
    data = json.loads(Path(batch_path).read_text(encoding="utf-8"))
    deliveries = data["deliveries"]

    by_buffer = {}
    for d in deliveries:
        if d.get("handlingMethod") != "hopper" or not d.get("hopper"):
            continue
        by_buffer.setdefault(d["buffer"], []).append(d)

    type_priority = {"S": 0, "M": 1, "L": 2, "W": 3}

    def sort_key(d):
        prefix, num = d["hopper"].split("-")
        return (type_priority[prefix], int(num))

    sequence = []
    for buf in BUFFER_ORDER:
        if buf not in by_buffer: continue
        for d in sorted(by_buffer[buf], key=sort_key):
            hid = hopper_name_to_id(d["hopper"])
            sequence.append({
                "buffer":    buf,
                "letter":    buf[0],
                "row":       int(buf[1]),
                "hopper":    d["hopper"],
                "hopper_id": hid,
                "count":     d["deliverQuantity"],
                "name":      d["name"],
                "article":   d["article"],
            })
    return sequence


def write_sequence_file(sequence, out_path="hopper_sequence.txt"):
    lines = ["# Generated hopper sequence",
             "# buffer hopper id count article name"]
    for s in sequence:
        lines.append(
            f"{s['buffer']:>3}  {s['hopper']:>5}  id={s['hopper_id']:<2}  "
            f"count={s['count']:<5}  {s['article']:<12}  {s['name']}"
        )
    Path(out_path).write_text("\n".join(lines), encoding="utf-8")
    print(f"Sequence written to {out_path}")


# ============================================================
def main():
    global current_row

    sequence = build_sequence(BATCH_FILE)
    write_sequence_file(sequence)

    print(f"\nTotal hopper commands: {len(sequence)}\n")
    for i, s in enumerate(sequence, 1):
        print(f"{i:2}. buf {s['buffer']}  {s['hopper']} (id={s['hopper_id']})  "
              f"count={s['count']}  --  {s['name']}")
    print("\nControls: s = skip current step, q = quit sequence\n")

    connect()

    try:
        conv = Conveyor()
    except Exception as e:
        print(f"!! Kon conveyor niet openen: {e}")
        ser.close()
        return

    if sequence:
        move_y_as(sequence[0]["letter"])

    skipped = []
    aborted = False

    try:
        for i, s in enumerate(sequence):
            # ---- ROW-WISSEL ? -> band X seconden vooruit ----
            if current_row is None:
                # eerste stap: box staat al op row 4 (door main_control)
                current_row = s["row"]
            elif s["row"] != current_row:
                rows_to_advance = current_row - s["row"]
                if rows_to_advance <= 0:
                    print(f"!! onverwachte rij-volgorde "
                          f"({current_row} -> {s['row']}), overslaan advance")
                else:
                    total = ROW_ADVANCE_SECONDS * rows_to_advance
                    ok = conveyor_run_for(
                        conv, total,
                        label=f"Advance {rows_to_advance} row(s) "
                              f"({current_row} -> {s['row']})"
                    )
                    if not ok:
                        aborted = True
                        break
                current_row = s["row"]

            if i > 0:
                if delay(DELAY_BETWEEN) == "quit":
                    aborted = True; break

            move_y_as(s["letter"])

            print(f"\n--- step {i+1}/{len(sequence)}  buf {s['buffer']}  "
                  f"{s['hopper']} (id={s['hopper_id']})  count={s['count']} ---")
            result = hopper(s["hopper_id"], s["count"])

            if result == "quit":
                aborted = True; break
            if result in ("skip", "timeout", "err"):
                skipped.append((s, result))

        # ---- BOX IS VOL -> uitvoeren ----
        if not aborted:
            print("\n[BOX] Alle rijen gevuld -- box uitvoeren ...")
            conveyor_run_for(conv, FINAL_EJECT_SECONDS, label="Final eject")
    finally:
        conv.close()
        ser.close()

    print("\n=========== summary ===========")
    print(f"Total steps   : {len(sequence)}")
    print(f"Skipped/failed: {len(skipped)}")
    for s, why in skipped:
        print(f"  - {s['buffer']} {s['hopper']} (id={s['hopper_id']}) [{why}]")
    print("Sequence was aborted by user (q)." if aborted else "Sequence finished.")


if __name__ == "__main__":
    main()
