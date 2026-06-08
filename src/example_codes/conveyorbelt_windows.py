"""
Mitsubishi FR-D720S Modbus RTU control example - Windows-compatible version.

Changes from Linux version:
  - PORT uses Windows COM port syntax (e.g. "COM3")
  - KeyListener rewritten using msvcrt (no termios/tty/select)
  - select module removed
"""

import sys
import time
import msvcrt
import minimalmodbus
import serial

# ---------------------------------------------------------------------------
# User configuration
# ---------------------------------------------------------------------------
PORT          = "COM5"          # <-- change to your COM port
SLAVE_ADDR    = 1
BAUDRATE      = 19200
PARITY        = serial.PARITY_EVEN
BYTESIZE      = 8
STOPBITS      = 1
TIMEOUT_S     = 0.4

MAX_FREQ_HZ   = 120
SPEED_PCT     = 10
RUN_TIME_S    = 5.0
POLL_PERIOD_S = 0.2

INTER_TX_S    = 0.02
MAX_RETRIES   = 3

MOTOR_POLES   = 4

# ---------------------------------------------------------------------------
# Register addresses (minimalmodbus uses 0-based: 4xxxx - 40001)
# ---------------------------------------------------------------------------
REG_RESET      = 40002 - 40001
REG_CONTROL    = 40009 - 40001
REG_SET_FREQ   = 40014 - 40001
REG_OUT_FREQ   = 40201 - 40001
REG_OUT_RPM    = 40208 - 40001

CTRL_STOP = 0x0001
CTRL_STF  = 0x0002
CTRL_STR  = 0x0004


# ---------------------------------------------------------------------------
# Non-blocking keyboard input (Windows - msvcrt)
# ---------------------------------------------------------------------------
class KeyListener:
    """
    Windows replacement for the termios/tty/select KeyListener.
    msvcrt.kbhit() checks if a key is waiting; msvcrt.getwch() reads it.
    No context-manager setup/teardown needed on Windows.
    """

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass  # nothing to restore on Windows

    def get_key(self):
        """Return a single character if one is waiting, else ''."""
        if msvcrt.kbhit():
            return msvcrt.getwch()
        return ''


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


# ---------------------------------------------------------------------------
def open_inverter() -> minimalmodbus.Instrument:
    inv = minimalmodbus.Instrument(PORT, SLAVE_ADDR, mode=minimalmodbus.MODE_RTU)
    inv.serial.baudrate = BAUDRATE
    inv.serial.parity   = PARITY
    inv.serial.bytesize = BYTESIZE
    inv.serial.stopbits = STOPBITS
    inv.serial.timeout  = TIMEOUT_S
    inv.clear_buffers_before_each_transaction = True
    time.sleep(0.2)
    return inv


def write_freq(inv, hz: float) -> None:
    value = int(round(hz * 100))
    _with_retry(inv, inv.write_register, REG_SET_FREQ, value, functioncode=6)


def set_control(inv, bits: int) -> None:
    _with_retry(inv, inv.write_register, REG_CONTROL, bits, functioncode=6)


def read_output_freq(inv) -> float:
    raw = _with_retry(inv, inv.read_register, REG_OUT_FREQ, functioncode=3)
    return raw / 100.0


def read_output_rpm(inv) -> int:
    return _with_retry(inv, inv.read_register, REG_OUT_RPM, functioncode=3)


# ---------------------------------------------------------------------------
# Control flow helpers
# ---------------------------------------------------------------------------
class QuitRequested(Exception):
    pass


def handle_keys(keys: KeyListener, inv) -> None:
    """
    Check stdin for 'p' (pause) or 'q' (quit).
    Blocks here while paused (motor stopped).
    """
    k = keys.get_key()
    if not k:
        return
    if k.lower() == 'q':
        raise QuitRequested()
    if k.lower() == 'p':
        print("\n[PAUSED] motor stopped. Press 'p' to resume, 'q' to quit.")
        set_control(inv, CTRL_STOP)
        while True:
            k2 = keys.get_key()
            if k2 and k2.lower() == 'p':
                print("[RESUMED]")
                return
            if k2 and k2.lower() == 'q':
                raise QuitRequested()
            time.sleep(0.05)


def wait_until_stopped(inv, keys: KeyListener, t_prev_ref: list,
                       revs_accum_ref: list) -> None:
    """Spin until output frequency is essentially zero."""
    while True:
        time.sleep(POLL_PERIOD_S)
        handle_keys(keys, inv)
        t_now = time.monotonic()
        dt = t_now - t_prev_ref[0]
        t_prev_ref[0] = t_now
        try:
            f_out = read_output_freq(inv)
        except Exception:
            continue
        revs_accum_ref[0] += (f_out / (MOTOR_POLES / 2.0)) * dt
        if f_out < 0.05:
            return


def run_direction(inv, keys: KeyListener, direction_bits: int,
                  label: str, cycle: int) -> float:
    target_hz = MAX_FREQ_HZ * SPEED_PCT / 100.0
    print(f"\n--- cycle {cycle}: {label} at {SPEED_PCT}% ({target_hz:.2f} Hz) ---")

    set_control(inv, direction_bits)

    revs = [0.0]
    t_start = time.monotonic()
    t_prev = [t_start]

    while True:
        time.sleep(POLL_PERIOD_S)
        handle_keys(keys, inv)
        t_now = time.monotonic()
        dt = t_now - t_prev[0]
        t_prev[0] = t_now

        try:
            f_out = read_output_freq(inv)
        except Exception as e:
            print(f"  [warn] read_output_freq failed: {e}")
            continue

        revs[0] += (f_out / (MOTOR_POLES / 2.0)) * dt

        try:
            rpm = read_output_rpm(inv)
        except Exception:
            rpm = -1

        print(f"  t={t_now - t_start:5.2f}s   f_out={f_out:5.2f} Hz   "
              f"rpm(reg)={rpm:5d}   est. revs={revs[0]:7.2f}")

        if t_now - t_start >= RUN_TIME_S:
            break

    set_control(inv, CTRL_STOP)
    print("  -> STOP, waiting for ramp-down")
    wait_until_stopped(inv, keys, t_prev, revs)

    print(f"  cycle {cycle} {label}: revolutions = {revs[0]:.2f}")
    return revs[0]


# ---------------------------------------------------------------------------
def main() -> None:
    inv = open_inverter()
    print(f"Connected to inverter at {PORT}, slave {SLAVE_ADDR}")
    print("Keys:  p = pause/resume,  q = quit")

    set_control(inv, CTRL_STOP)
    time.sleep(0.2)

    target_hz = MAX_FREQ_HZ * SPEED_PCT / 100.0
    write_freq(inv, target_hz)

    cycle = 0
    totals = {"fwd": 0.0, "rev": 0.0}
    per_cycle = []

    with KeyListener() as keys:
        try:
            while True:
                cycle += 1
                fwd = run_direction(inv, keys, CTRL_STF, "FORWARD", cycle)
                time.sleep(1.0)
                rev = run_direction(inv, keys, CTRL_STR, "REVERSE", cycle)

                totals["fwd"] += fwd
                totals["rev"] += rev
                per_cycle.append((cycle, fwd, rev, fwd - rev))

                print(f"\n=== cycle {cycle} summary ===")
                print(f"  fwd={fwd:7.2f}  rev={rev:7.2f}  net={fwd - rev:+7.2f}")
                print(f"  cumulative: fwd={totals['fwd']:8.2f}  "
                      f"rev={totals['rev']:8.2f}  "
                      f"net={totals['fwd'] - totals['rev']:+8.2f}")
                print(f"  comms: ok={CommStats.ok}  retried={CommStats.retried}  "
                      f"failed={CommStats.failed}")

                time.sleep(1.0)

        except (QuitRequested, KeyboardInterrupt):
            print("\n[QUIT] stopping motor...")
        finally:
            try:
                set_control(inv, CTRL_STOP)
            except Exception:
                pass

    # Final report
    print("\n=============== FINAL REPORT ===============")
    print(f"Cycles completed: {len(per_cycle)}")
    print(f"{'cyc':>4} {'fwd':>9} {'rev':>9} {'net':>9}")
    for c, f, r, n in per_cycle:
        print(f"{c:>4} {f:>9.2f} {r:>9.2f} {n:>+9.2f}")
    if per_cycle:
        fwds = [f for _, f, _, _ in per_cycle]
        revs = [r for _, _, r, _ in per_cycle]
        nets = [n for _, _, _, n in per_cycle]

        def stats(name, xs):
            mean = sum(xs) / len(xs)
            spread = max(xs) - min(xs)
            var = sum((x - mean) ** 2 for x in xs) / len(xs)
            sd = var ** 0.5
            print(f"  {name}: mean={mean:8.2f}  min={min(xs):8.2f}  "
                  f"max={max(xs):8.2f}  spread={spread:6.2f}  sd={sd:6.3f}")

        print("\nRepeatability:")
        stats("fwd", fwds)
        stats("rev", revs)
        stats("net", nets)

    print(f"\nComms totals: ok={CommStats.ok}  "
          f"retried={CommStats.retried}  failed={CommStats.failed}")


if __name__ == "__main__":
    main()
