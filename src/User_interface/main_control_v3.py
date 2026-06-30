"""
main_control.py  (v7)
---------------------
Robot sequence controller (UI-PC side, Windows).

Wijziging t.o.v. v6:
- Fix UnicodeEncodeError in sma3-prompt-input thread op Windows-consoles
  die cp1252 gebruiken. We forceren nu UTF-8 op stdout/stderr en hebben
  een _safe_print fallback die naar ASCII degradeert als de console
  bepaalde Unicode-tekens (zoals box-drawing) niet aankan.
"""

import json
import os
import sys
import time
import threading
import queue
import subprocess
from pathlib import Path

# ── Console-encoding fix (Windows cp1252 -> UTF-8) ──────────────────────
# Moet vóór de eerste print/Unicode-output gebeuren.
for _stream_name in ("stdout", "stderr"):
    _stream = getattr(sys, _stream_name, None)
    if _stream is not None and hasattr(_stream, "reconfigure"):
        try:
            _stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


def _safe_print(*args, **kwargs):
    """print() die nooit crasht op UnicodeEncodeError.
    Bij encoding-probleem valt hij terug op ASCII met '?' voor onbekende
    tekens, zodat de thread blijft draaien."""
    try:
        print(*args, **kwargs)
    except UnicodeEncodeError:
        sep = kwargs.get("sep", " ")
        end = kwargs.get("end", "\n")
        text = sep.join(str(a) for a in args) + end
        try:
            enc = (sys.stdout.encoding or "ascii")
        except Exception:
            enc = "ascii"
        sys.stdout.write(text.encode(enc, "replace").decode(enc, "replace"))
        if kwargs.get("flush"):
            try:
                sys.stdout.flush()
            except Exception:
                pass


from comm import Link

BASE_DIR          = Path(__file__).resolve().parent
ACTIVE_BATCH_FILE = BASE_DIR / "data" / "active_batch.json"
CONVERTED_FILE    = BASE_DIR / "data" / "robot_jobs.json"

# ── SMA3 control channel ────────────────────────────────────────────────
SMA3_CONTROL_HOST    = "192.168.108.43"
SMA3_CONTROL_PORT    = 9100
SMA3_CONNECT_TIMEOUT = 30

DEFAULT_RETRIES         = 3
DEFAULT_DOOSAN_SEQUENCE = "pick_and_place"
WAIT_SECONDS            = 10

# ════════════════════════════════════════════════════════════════════════
#  CONVEYOR CONFIG  (Windows COM-poorten)
# ════════════════════════════════════════════════════════════════════════
INV_PORT     = "COM5"
ARDUINO_PORT = "COM7"

SLAVE_ADDR   = 1
BAUDRATE     = 19200
BYTESIZE     = 8
STOPBITS     = 1
TIMEOUT_S    = 0.4

MAX_FREQ_HZ   = 120.0
SPEED_PCT     = 3
POLL_PERIOD_S = 0.05
INTER_TX_S    = 0.02
MAX_RETRIES   = 3

ARDUINO_BAUD    = 115200
ARDUINO_TIMEOUT = 0.2

REG_CONTROL  = 40009 - 40001
REG_SET_FREQ = 40014 - 40001
REG_OUT_FREQ = 40201 - 40001
CTRL_STOP    = 0x0001
CTRL_STF     = 0x0002
CTRL_STR     = 0x0004

CONVEYOR_BOX_TIMEOUT_S = 60.0

# Sensor-index die bepaalt dat de box op de eerste rij (row 4) staat.
INTAKE_SENSOR_IDX = 0

# ════════════════════════════════════════════════════════════════════════
#  HOPPER-SEQUENCER subprocess
# ════════════════════════════════════════════════════════════════════════
HOPPER_SEQUENCER_SCRIPT  = BASE_DIR / "main_control_hoppers_v3.py"
COM_PORT_RELEASE_DELAY_S = 1.0


# ════════════════════════════════════════════════════════════════════════
#  COMMUNICATIE-STATISTIEKEN
# ════════════════════════════════════════════════════════════════════════
class CommStats:
    ok      = 0
    retried = 0
    failed  = 0


# ════════════════════════════════════════════════════════════════════════
#  HELPERS — batch -> jobs
# ════════════════════════════════════════════════════════════════════════
def wait_for_start(seconds: int = WAIT_SECONDS) -> None:
    _safe_print(f"[MAIN_CONTROL] Wachten {seconds} seconden voor start sequence...",
                flush=True)
    for remaining in range(seconds, 0, -1):
        sys.stdout.write(f"\r[MAIN_CONTROL] Start over {remaining:2d}s ")
        sys.stdout.flush()
        time.sleep(1)
    sys.stdout.write("\r[MAIN_CONTROL] Sequence gestart!          \n")
    sys.stdout.flush()


def load_active_batch(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(
            f"active_batch.json niet gevonden op {path}. "
            "Start eerst een batch via app.py."
        )
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def is_robotarm_delivery(delivery: dict) -> bool:
    method = (delivery.get("handlingMethod") or "").lower()
    if method == "robotarm":
        return True
    if method == "hopper":
        return False
    return delivery.get("hopper") in (None, "", False)


def has_hopper_delivery(active_batch: dict) -> bool:
    for d in active_batch.get("deliveries", []):
        method = (d.get("handlingMethod") or "").lower()
        if method == "hopper" or d.get("hopper"):
            return True
    return False


def convert_active_batch(active_batch: dict) -> list:
    jobs = []
    for delivery in active_batch.get("deliveries", []):
        if not is_robotarm_delivery(delivery):
            continue
        buffer = delivery.get("buffer")
        if not buffer:
            continue
        part_name = (
            delivery.get("name")
            or delivery.get("article")
            or f"part_{delivery.get('partId')}"
        )
        jobs.append({
            "tray":            buffer,
            "part":            part_name,
            "count":           int(delivery.get("deliverQuantity") or 0),
            "retries":         DEFAULT_RETRIES,
            "drop_pose":       buffer,
            "doosan_sequence": DEFAULT_DOOSAN_SEQUENCE,
        })
    return jobs


def save_json(data, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


# ════════════════════════════════════════════════════════════════════════
#  CONVEYOR — Modbus inverter + Arduino sensoren (multi-sensor)
# ════════════════════════════════════════════════════════════════════════
class _SensorReader(threading.Thread):
    """
    Leest sensorlijnen "S<idx>:<0|1>" van de Arduino.
    Houdt per sensor-index state bij en publiceert rising edges.
    Andere regels (Y-as info, Gate OPEN, etc.) worden genegeerd.
    """
    def __init__(self, port: str, baud: int, timeout: float):
        super().__init__(daemon=True)
        import serial as pyserial
        self.ser = pyserial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self.state: dict[int, int] = {}
        self.rising_edges: "queue.Queue[tuple[int,float]]" = queue.Queue()
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def run(self) -> None:
        prev: dict[int, int] = {}
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue
            # Verwacht: S<idx>:<0|1>
            if (len(line) >= 4 and line[0] == "S" and line[1].isdigit()
                    and line[2] == ":" and line[3] in ("0", "1")):
                idx = int(line[1])
                cur = 1 if line[3] == "1" else 0
                with self._lock:
                    self.state[idx] = cur
                    p = prev.get(idx, 0)
                    if p == 0 and cur == 1:
                        self.rising_edges.put((idx, time.monotonic()))
                    prev[idx] = cur

    def consume_edges(self) -> None:
        while not self.rising_edges.empty():
            try:
                self.rising_edges.get_nowait()
            except queue.Empty:
                break

    def got_rising_edge(self, sensor_idx: int) -> bool:
        """Pop één rising edge van de gevraagde sensor; True als gevonden."""
        leftover = []
        found = False
        while True:
            try:
                idx, ts = self.rising_edges.get_nowait()
            except queue.Empty:
                break
            if idx == sensor_idx and not found:
                found = True
            else:
                leftover.append((idx, ts))
        for item in leftover:
            self.rising_edges.put(item)
        return found

    def get_state(self, sensor_idx: int) -> int:
        with self._lock:
            return self.state.get(sensor_idx, 0)

    def stop(self) -> None:
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


def _with_retry(inv, func, *args, **kwargs):
    import minimalmodbus
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


def _open_inverter():
    import serial as pyserial
    import minimalmodbus
    inv = minimalmodbus.Instrument(INV_PORT, SLAVE_ADDR,
                                   mode=minimalmodbus.MODE_RTU)
    inv.serial.baudrate = BAUDRATE
    inv.serial.parity   = pyserial.PARITY_EVEN
    inv.serial.bytesize = BYTESIZE
    inv.serial.stopbits = STOPBITS
    inv.serial.timeout  = TIMEOUT_S
    inv.clear_buffers_before_each_transaction = True
    time.sleep(0.2)
    return inv


def advance_conveyor() -> bool:
    """Stuur band tot S0 (intake / row 4) een rising edge geeft."""
    _safe_print(f"\n[CONVEYOR] Start advance (wacht op S{INTAKE_SENSOR_IDX})...",
                flush=True)
    sensor  = None
    inv     = None
    running = False

    def stop_belt(reason: str = "") -> None:
        nonlocal running
        if inv is not None:
            try:
                _with_retry(inv, inv.write_register, REG_CONTROL,
                            CTRL_STOP, functioncode=6)
            except Exception as e:
                _safe_print(f"[CONVEYOR] Kon STOP-command niet sturen: {e}",
                            flush=True)
        running = False
        if reason:
            _safe_print(f"[CONVEYOR] stopped ({reason})", flush=True)
        else:
            _safe_print("[CONVEYOR] stopped", flush=True)

    try:
        _safe_print(f"[CONVEYOR] Inverter openen op {INV_PORT} ...", flush=True)
        inv = _open_inverter()

        _safe_print(f"[CONVEYOR] Arduino openen op {ARDUINO_PORT} ...", flush=True)
        sensor = _SensorReader(ARDUINO_PORT, ARDUINO_BAUD, ARDUINO_TIMEOUT)
        sensor.start()

        target_hz = MAX_FREQ_HZ * SPEED_PCT / 100.0

        _with_retry(inv, inv.write_register, REG_CONTROL, CTRL_STOP,
                    functioncode=6)
        time.sleep(0.2)
        _with_retry(inv, inv.write_register, REG_SET_FREQ,
                    int(round(target_hz * 100)), functioncode=6)

        sensor.consume_edges()
        # Als S0 al HIGH is bij start, wachten tot LOW (armed) zodat
        # we alleen een NIEUWE doos detecteren.
        armed = (sensor.get_state(INTAKE_SENSOR_IDX) == 0)

        _with_retry(inv, inv.write_register, REG_CONTROL, CTRL_STF,
                    functioncode=6)
        running = True
        _safe_print(f"[CONVEYOR] Band loopt op {target_hz:.2f} Hz "
                    f"(armed={armed}), wachten op doos op S{INTAKE_SENSOR_IDX}...",
                    flush=True)

        t_start = time.monotonic()
        while True:
            time.sleep(POLL_PERIOD_S)
            if sensor.get_state(INTAKE_SENSOR_IDX) == 0:
                armed = True
            if armed and sensor.got_rising_edge(INTAKE_SENSOR_IDX):
                try:
                    raw = _with_retry(inv, inv.read_register,
                                      REG_OUT_FREQ, functioncode=3)
                    f_out = raw / 100.0
                except Exception:
                    f_out = float("nan")
                _safe_print(f"[CONVEYOR] Doos gedetecteerd op S{INTAKE_SENSOR_IDX} "
                            f"(f_out={f_out:.2f} Hz).", flush=True)
                stop_belt("box detected")
                _safe_print(f"[CONVEYOR] Comms: ok={CommStats.ok}  "
                            f"retried={CommStats.retried}  "
                            f"failed={CommStats.failed}", flush=True)
                return True

            if time.monotonic() - t_start > CONVEYOR_BOX_TIMEOUT_S:
                stop_belt(f"timeout na {CONVEYOR_BOX_TIMEOUT_S:.0f}s "
                          f"zonder doos")
                return False

    except Exception as e:
        _safe_print(f"[CONVEYOR] Fout: {e}", flush=True)
        try:
            stop_belt("error")
        except Exception:
            pass
        return False
    finally:
        if sensor is not None:
            sensor.stop()
        if inv is not None:
            try:
                inv.serial.close()
            except Exception:
                pass
        _safe_print("[CONVEYOR] Klaar.\n", flush=True)


# ════════════════════════════════════════════════════════════════════════
#  HOPPER-SEQUENCER subprocess  (REAL-TIME OUTPUT)
# ════════════════════════════════════════════════════════════════════════
def run_hopper_sequence() -> bool:
    if not HOPPER_SEQUENCER_SCRIPT.exists():
        _safe_print(f"[MAIN_CONTROL] Hopper-script niet gevonden: "
                    f"{HOPPER_SEQUENCER_SCRIPT}", flush=True)
        return False

    _safe_print(f"[MAIN_CONTROL] COM-poorten {COM_PORT_RELEASE_DELAY_S:.1f}s "
                f"laten rusten ...", flush=True)
    time.sleep(COM_PORT_RELEASE_DELAY_S)

    _safe_print(f"[MAIN_CONTROL] Start hopper-sequence: "
                f"{HOPPER_SEQUENCER_SCRIPT.name}", flush=True)
    _safe_print("=" * 60, flush=True)

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    # Zorg dat het subprocess ook UTF-8 stdout heeft.
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"

    try:
        proc = subprocess.Popen(
            [sys.executable, "-u", str(HOPPER_SEQUENCER_SCRIPT)],
            cwd=str(BASE_DIR),
            env=env,
        )
        rc = proc.wait()
    except Exception as e:
        _safe_print(f"[MAIN_CONTROL] Kon hopper-sequencer niet starten: {e}",
                    flush=True)
        return False

    _safe_print("=" * 60, flush=True)
    _safe_print(f"[MAIN_CONTROL] Hopper-sequence klaar (exit code {rc}).",
                flush=True)
    return rc == 0


# ════════════════════════════════════════════════════════════════════════
#  SMA3
# ════════════════════════════════════════════════════════════════════════
def run_sma3_sequence(jobs: list) -> int:
    done_event  = threading.Event()
    error_event = threading.Event()
    last_error  = {"msg": None}

    # Single worker thread that reads stdin so we never have two input()s
    # competing if prompts come back-to-back.
    prompt_queue: "list[tuple[int, tuple[str, ...], str]]" = []
    prompt_cv = threading.Condition()

    # Box-drawing characters; vallen automatisch terug op ASCII als de
    # console UTF-8 niet aankan (via _safe_print).
    BOX_TOP    = "┌" + ("─" * 58)
    BOX_BOTTOM = "└" + ("─" * 58)

    def _prompt_worker():
        while True:
            with prompt_cv:
                while not prompt_queue and not done_event.is_set():
                    prompt_cv.wait(timeout=0.5)
                if done_event.is_set() and not prompt_queue:
                    return
                pid, allowed, message = prompt_queue.pop(0)

            allowed_str = "/".join(allowed)
            _safe_print("", flush=True)
            _safe_print(BOX_TOP, flush=True)
            _safe_print(f"│ [SMA3 PROMPT #{pid}] {message}", flush=True)
            _safe_print(f"│ Antwoorden: {allowed_str}", flush=True)
            _safe_print(BOX_BOTTOM, flush=True)

            while True:
                try:
                    sys.stdout.write(f"  >>> antwoord ({allowed_str}): ")
                    sys.stdout.flush()
                    ans = sys.stdin.readline()
                except (EOFError, KeyboardInterrupt):
                    ans = "abort"
                if ans is None:
                    ans = "abort"
                ans = ans.strip().lower()
                if ans in allowed:
                    break
                _safe_print(f"  (ongeldig, kies een van: {allowed_str})",
                            flush=True)

            try:
                link.send(json.dumps({
                    "cmd": "prompt_reply",
                    "id":  pid,
                    "answer": ans,
                }))
                _safe_print(f"  -> antwoord {ans!r} verstuurd naar SMA3.",
                            flush=True)
            except Exception as e:
                _safe_print(f"  !! kon antwoord niet sturen: {e}", flush=True)

    def on_message(msg: str) -> None:
        line = msg.strip()
        low  = line.lower()

        # --- New: operator prompt from SMA3 ---
        if line.startswith("Prompt:"):
            try:
                # "Prompt: <id> | <allowed_csv> | <message>"
                payload = line[len("Prompt:"):].strip()
                pid_str, allowed_csv, message = [p.strip()
                                                 for p in payload.split("|", 2)]
                pid = int(pid_str)
                allowed = tuple(a.strip().lower()
                                for a in allowed_csv.split(",") if a.strip())
            except Exception as e:
                _safe_print(f"[SMA3] !! kon prompt niet parsen: {line!r} ({e})",
                            flush=True)
                return
            with prompt_cv:
                prompt_queue.append((pid, allowed, message))
                prompt_cv.notify()
            return

        # --- Original behaviour ---
        _safe_print(f"[SMA3] {line}", flush=True)
        if low.startswith("done"):
            done_event.set()
        elif low.startswith("error"):
            last_error["msg"] = line
            error_event.set()
            done_event.set()

    link = Link(role="client",
                host=SMA3_CONTROL_HOST,
                port=SMA3_CONTROL_PORT,
                on_message=on_message)

    _safe_print(f"[MAIN_CONTROL] Verbinden met SMA3 op "
                f"{SMA3_CONTROL_HOST}:{SMA3_CONTROL_PORT} ...", flush=True)
    link.start()
    if not link.wait_until_connected(timeout=SMA3_CONNECT_TIMEOUT):
        _safe_print(f"[MAIN_CONTROL] Kon SMA3 niet bereiken binnen "
                    f"{SMA3_CONNECT_TIMEOUT}s.", flush=True)
        link.stop()
        return 1
    _safe_print("[MAIN_CONTROL] Verbonden met SMA3.", flush=True)

    worker = threading.Thread(target=_prompt_worker,
                              name="sma3-prompt-input", daemon=True)
    worker.start()

    payload = {"cmd": "start_sequence", "jobs": jobs}
    link.send(json.dumps(payload, ensure_ascii=False))
    _safe_print(f"[MAIN_CONTROL] {len(jobs)} job(s) verstuurd. "
                f"Wachten op SMA3 (Ctrl+C om af te breken)...", flush=True)

    try:
        while not done_event.is_set():
            if not link.is_connected():
                _safe_print("[MAIN_CONTROL] Verbinding met SMA3 verbroken.",
                            flush=True)
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        _safe_print("\n[MAIN_CONTROL] Ctrl+C ontvangen, abort sturen...",
                    flush=True)
        try:
            link.send(json.dumps({"cmd": "abort"}))
        except Exception:
            pass
        time.sleep(0.5)

    # Wake the prompt worker so it can exit.
    with prompt_cv:
        prompt_cv.notify_all()

    link.stop()

    if error_event.is_set():
        _safe_print(f"[MAIN_CONTROL] SMA3 meldde fout: {last_error['msg']}",
                    flush=True)
        return 2
    if not done_event.is_set():
        return 1

    _safe_print("[MAIN_CONTROL] SMA3 sequence afgerond.", flush=True)
    return 0
# ════════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════════
def main() -> int:
    wait_for_start(WAIT_SECONDS)

    try:
        active_batch = load_active_batch(ACTIVE_BATCH_FILE)
    except FileNotFoundError as err:
        _safe_print(f"[MAIN_CONTROL] {err}", flush=True)
        return 1
    except json.JSONDecodeError as err:
        _safe_print(f"[MAIN_CONTROL] active_batch.json is corrupt: {err}",
                    flush=True)
        return 1

    total_deliveries = len(active_batch.get("deliveries", []))
    jobs = convert_active_batch(active_batch)
    hopper_present = has_hopper_delivery(active_batch)
    _safe_print(f"[MAIN_CONTROL] {len(jobs)} robotarm job(s) "
                f"uit {total_deliveries} delivery(s). "
                f"Hopper-deliveries aanwezig: {hopper_present}", flush=True)

    _safe_print("[MAIN_CONTROL] Geconverteerd robot job-bestand:", flush=True)
    _safe_print(json.dumps(jobs, indent=2, ensure_ascii=False), flush=True)
    save_json(jobs, CONVERTED_FILE)
    _safe_print(f"[MAIN_CONTROL] Opgeslagen kopie: {CONVERTED_FILE}", flush=True)

    if jobs:
        rc = run_sma3_sequence(jobs)
        if rc != 0:
            return rc
    else:
        _safe_print("[MAIN_CONTROL] Geen robotarm-taken. "
                    "SMA3-stap wordt overgeslagen.", flush=True)

    if not hopper_present:
        _safe_print("[MAIN_CONTROL] Geen hopper-deliveries. Klaar.", flush=True)
        return 0

    if not advance_conveyor():
        _safe_print("[MAIN_CONTROL] Conveyor advance mislukt of timeout. "
                    "Hopper-sequence wordt overgeslagen.", flush=True)
        return 2

    _safe_print("[MAIN_CONTROL] Doos op row 4. Start hopper-sequence.",
                flush=True)

    if not run_hopper_sequence():
        _safe_print("[MAIN_CONTROL] Hopper-sequence mislukt.", flush=True)
        return 2

    _safe_print("[MAIN_CONTROL] Alles klaar.", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        _safe_print("\n[MAIN_CONTROL] Onderbroken.", flush=True)
        sys.exit(130)
