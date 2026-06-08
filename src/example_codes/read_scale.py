#!/usr/bin/env python3
import curses
import re
import serial
import threading

PORT = "/dev/ttyUSB0"
BAUD = 9600

# Continuous output: ASNG/W+ 0.080  kg
PATTERN_CONT = re.compile(r"([A-Z/+\-]+)\s+([\+\-]?\s*[\d.]+)\s*(kg|g|lb)", re.IGNORECASE)

# Command responses: + 0.000 kg  /  - 0.500 kg
PATTERN_RESP = re.compile(r"^([\+\-])\s*([\d.]+)\s*(kg|g|lb)", re.IGNORECASE)

# Acknowledgements: <Z  <T  <H
PATTERN_ACK = re.compile(r"^<([ZTH])$")

COMMANDS = {
    "Z": "Tare/Zero",
    "H": "Hold",
    "N": "Request net weight",
    "G": "Request gross weight",
    "T": "Request tare weight",
}

HELP = "  ".join(f"[{k}] {v}" for k, v in COMMANDS.items())


def run(stdscr, ser):
    curses.curs_set(1)
    stdscr.nodelay(False)
    height, width = stdscr.getmaxyx()

    # Layout:
    #  0      Weight (live)
    #  1      Help
    #  2      ─── divider
    #  3      Last N/G/T response
    #  4      ─── divider
    #  5..h-2 Scrolling log
    #  h-1    Input

    LOG_TOP = 5
    LOG_BOT = height - 2
    log_lines = []
    last_cmd = [None]
    cmd_buf = []
    lock = threading.Lock()
    stop_event = threading.Event()

    def draw_static():
        stdscr.addstr(1, 0, HELP[:width - 1])
        stdscr.addstr(2, 0, "─" * (width - 1))
        stdscr.addstr(3, 0, "Response: —")
        stdscr.addstr(4, 0, "─" * (width - 1))
        stdscr.addstr(height - 1, 0, "> ")
        stdscr.refresh()

    def redraw_weight(text):
        stdscr.move(0, 0); stdscr.clrtoeol()
        stdscr.addstr(0, 0, text[:width - 1], curses.A_BOLD)

    def redraw_response(text):
        stdscr.move(3, 0); stdscr.clrtoeol()
        stdscr.addstr(3, 0, f"Response: {text}"[:width - 1], curses.A_BOLD)

    def redraw_log():
        visible = log_lines[-(LOG_BOT - LOG_TOP + 1):]
        for i in range(LOG_BOT - LOG_TOP + 1):
            stdscr.move(LOG_TOP + i, 0); stdscr.clrtoeol()
            if i < len(visible):
                stdscr.addstr(LOG_TOP + i, 0, visible[i][:width - 1])

    def log(msg):
        log_lines.append(msg)
        redraw_log()

    def restore_cursor():
        stdscr.move(height - 1, 2 + len(cmd_buf))
        stdscr.refresh()

    def handle_line(line):
        line = line.strip()
        if not line:
            return

        # Acknowledgement: <Z, <T, <H
        m = PATTERN_ACK.match(line)
        if m:
            log(f"✓ Command <{m.group(1)}> acknowledged")
            return

        # Named response: N/W, G/W, T/W
        m = PATTERN_RESP.search(line)
        if m:
            sign = m.group(1)
            value = float(m.group(2))
            unit = m.group(3).lower()
            label = last_cmd[0] or "?"
            redraw_response(f"{label}: {sign}{value} {unit}")
            return

        # Continuous weight
        m = PATTERN_CONT.search(line)
        if m:
            status, value, unit = m.group(1), float(m.group(2).replace(" ", "")), m.group(3).lower()
            redraw_weight(f"Weight: [{status}] {value} {unit}")
            return

        log(f"(unparsed) {line}")

    def reader():
        while not stop_event.is_set():
            line = ser.readline().decode("ascii", errors="replace")
            if not line:
                continue
            with lock:
                handle_line(line)
                restore_cursor()

    draw_static()
    t = threading.Thread(target=reader, daemon=True)
    t.start()

    try:
        while True:
            ch = stdscr.get_wch()
            with lock:
                if ch in ("\n", "\r"):
                    cmd = "".join(cmd_buf).strip().upper()
                    cmd_buf.clear()
                    stdscr.move(height - 1, 2); stdscr.clrtoeol()
                    if cmd in COMMANDS:
                        ser.write(f"{cmd}\r\n".encode("ascii"))
                        last_cmd[0] = cmd
                        log(f">> {cmd}: {COMMANDS[cmd]}")
                    elif cmd:
                        log(f"Unknown '{cmd}'. Valid: {', '.join(COMMANDS)}")
                elif ch in (curses.KEY_BACKSPACE, "\x7f", "\b"):
                    if cmd_buf:
                        cmd_buf.pop()
                        stdscr.move(height - 1, 2); stdscr.clrtoeol()
                        stdscr.addstr(height - 1, 2, "".join(cmd_buf))
                elif isinstance(ch, str) and ch.isprintable():
                    cmd_buf.append(ch)
                    stdscr.addstr(height - 1, 2 + len(cmd_buf) - 1, ch)
                restore_cursor()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()


def main():
    with serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=False, rtscts=False, dsrdtr=False,
        timeout=1,
    ) as ser:
        curses.wrapper(run, ser)


if __name__ == "__main__":
    main()
