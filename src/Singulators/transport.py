import json
import os
import sys
import time


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ACTIVE_BATCH_FILE = os.path.join(BASE_DIR, "data", "active_batch.json")
DEFAULT_TRANSPORT_STATUS_FILE = os.path.join(BASE_DIR, "data", "transport_status.json")
ARDUINO_IDS = [1, 2, 3]
HOPPERS_PER_ARDUINO = 12
BAUDRATE = 115200
COMMAND_TIMEOUT_SECONDS = 30
SERIAL_TIMEOUT_SECONDS = COMMAND_TIMEOUT_SECONDS + 5
DISCOVERY_TIMEOUT_SECONDS = 1
ARDUINO_RESET_SECONDS = 2
DRY_RUN_VALUES = {"1", "true", "yes", "on"}


def hopper_id_to_number(hopper_id):
    if not hopper_id or "-" not in hopper_id:
        return None

    prefix, raw_index = hopper_id.split("-", 1)
    try:
        index = int(raw_index)
    except ValueError:
        return None

    offsets = {
        "S": 0,
        "M": 5,
        "L": 10,
        "W": 15,
    }
    offset = offsets.get(prefix.upper())
    if offset is None:
        return None
    return offset + index


def arduino_for_hopper(hopper_number):
    if hopper_number is None or hopper_number < 1:
        return None
    return ((hopper_number - 1) // HOPPERS_PER_ARDUINO) + 1


def parse_arduino_ports():
    raw = os.environ.get("ARDUINO_PORTS", "").strip()
    ports = {}
    if not raw:
        return ports

    for item in raw.split(","):
        if "=" not in item:
            continue
        raw_id, port = item.split("=", 1)
        try:
            arduino_id = int(raw_id.strip())
        except ValueError:
            continue
        ports[arduino_id] = port.strip()
    return ports


def env_flag(name):
    return os.environ.get(name, "").strip().lower() in DRY_RUN_VALUES


def describe_serial_port(port_info):
    description = getattr(port_info, "description", "") or ""
    device = getattr(port_info, "device", "") or str(port_info)
    return f"{device} ({description})" if description else device


class ArduinoTransport:
    def __init__(self):
        self.ports = parse_arduino_ports()
        self.dry_run_enabled = env_flag("TRANSPORT_DRY_RUN")
        self.serial_connections = {}
        self.serial_module = None
        self.list_ports_module = None
        if self.dry_run_enabled:
            print("[TRANSPORT] TRANSPORT_DRY_RUN is enabled. Running in dry-run mode.")
            return
        try:
            import serial
            from serial.tools import list_ports

            self.serial_module = serial
            self.list_ports_module = list_ports
        except ImportError:
            print("[TRANSPORT] pyserial is not installed. Real Arduino transport is unavailable.")
            self.ports = {}

    @property
    def dry_run(self):
        return self.dry_run_enabled

    def open_serial(self, port, timeout=SERIAL_TIMEOUT_SECONDS):
        return self.serial_module.Serial(
            port=port,
            baudrate=BAUDRATE,
            timeout=timeout,
            write_timeout=timeout,
        )

    def send_on_connection(self, connection, command):
        connection.reset_input_buffer()
        connection.write((command + "\n").encode("utf-8"))
        connection.flush()
        return connection.readline().decode("utf-8", errors="replace").strip().lower()

    def discover_ports(self):
        if not self.list_ports_module:
            return {}

        discovered = {}
        serial_ports = list(self.list_ports_module.comports())
        if not serial_ports:
            print("[TRANSPORT] No serial ports found.")
            return discovered

        print("[TRANSPORT] ARDUINO_PORTS not configured. Scanning serial ports for Arduinos.")
        for port_info in serial_ports:
            port = port_info.device
            try:
                connection = self.open_serial(port, timeout=DISCOVERY_TIMEOUT_SECONDS)
            except OSError as error:
                print(f"[TRANSPORT] Could not open {describe_serial_port(port_info)}: {error}")
                continue

            try:
                time.sleep(ARDUINO_RESET_SECONDS)
                for arduino_id in ARDUINO_IDS:
                    if arduino_id in discovered:
                        continue
                    command = f"{arduino_id}-alive"
                    response = self.send_on_connection(connection, command)
                    if response == "success":
                        discovered[arduino_id] = port
                        print(f"[TRANSPORT] Arduino {arduino_id} detected on {port}.")
                        break
            finally:
                connection.close()

        if not discovered:
            print("[TRANSPORT] No Arduino controllers responded.")
        return discovered

    def open(self):
        if self.dry_run:
            return True

        if self.serial_module is None:
            print("[TRANSPORT] No serial module available.")
            return False

        if not self.ports:
            self.ports = self.discover_ports()

        if not self.ports:
            print("[TRANSPORT] No Arduino serial ports available.")
            return False

        for arduino_id in ARDUINO_IDS:
            port = self.ports.get(arduino_id)
            if not port:
                print(f"[TRANSPORT] Arduino {arduino_id} has no configured serial port.")
                continue
            try:
                self.serial_connections[arduino_id] = self.open_serial(port)
                time.sleep(ARDUINO_RESET_SECONDS)
                print(f"[TRANSPORT] Arduino {arduino_id} connected on {port}.")
            except OSError as error:
                print(f"[TRANSPORT] Could not connect Arduino {arduino_id} on {port}: {error}")
        return bool(self.serial_connections)

    def close(self):
        for connection in self.serial_connections.values():
            connection.close()

    def send(self, arduino_id, command):
        print(f"[TRANSPORT] -> {command}")
        if self.dry_run:
            print(f"[TRANSPORT] <- success (dry-run arduino {arduino_id})")
            return "success"

        connection = self.serial_connections.get(arduino_id)
        if not connection:
            print(f"[TRANSPORT] <- fail (arduino {arduino_id} not connected)")
            return "fail"

        response = self.send_on_connection(connection, command)
        if response not in {"success", "fail"}:
            print(f"[TRANSPORT] <- fail (unexpected response: {response or 'empty'})")
            return "fail"
        print(f"[TRANSPORT] <- {response}")
        return response

    def handshake(self, arduino_ids=None):
        arduino_ids = list(arduino_ids or ARDUINO_IDS)
        results = []
        for arduino_id in arduino_ids:
            result = self.send(arduino_id, f"{arduino_id}-alive")
            results.append(result == "success")
        if all(results):
            print("[TRANSPORT] arduino's alive.")
            return True
        print("[TRANSPORT] one or more arduino's did not respond with success.")
        return False


def load_active_batch(path):
    with open(path, encoding="utf-8") as file:
        return json.load(file)


def write_transport_status(active_batch, commands, status, message="", current_index=None):
    path = os.environ.get("TRANSPORT_STATUS_FILE", DEFAULT_TRANSPORT_STATUS_FILE)
    existing_events = []
    operator_note = ""
    if os.path.exists(path):
        try:
            with open(path, encoding="utf-8") as file:
                existing_status = json.load(file)
                existing_events = existing_status.get("events", [])
                operator_note = existing_status.get("operatorNote", "")
        except (OSError, json.JSONDecodeError):
            existing_events = []
    payload = {
        "status": status,
        "message": message,
        "updatedAt": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "events": existing_events[-400:],
        "operatorNote": operator_note,
        "batchId": (active_batch.get("batch") or {}).get("id"),
        "batchName": (active_batch.get("batch") or {}).get("name"),
        "currentIndex": current_index,
        "totalCommands": len(commands),
        "completedCommands": sum(1 for item in commands if item.get("status") == "success"),
        "failedCommands": sum(1 for item in commands if item.get("status") == "fail"),
        "commands": [
            {
                "index": index,
                "arduinoId": item.get("arduinoId"),
                "hopperNumber": item.get("hopperNumber"),
                "hopper": (item.get("assignment") or {}).get("hopper"),
                "buffer": (item.get("assignment") or {}).get("buffer"),
                "article": (item.get("assignment") or {}).get("article"),
                "quantity": item.get("quantity"),
                "command": item.get("command"),
                "status": item.get("status", "waiting"),
            }
            for index, item in enumerate(commands)
        ],
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        json.dump(payload, file, indent=2, ensure_ascii=False)


def build_hopper_commands(active_batch):
    commands = []
    for assignment in active_batch.get("hopperAssignments", []):
        hopper_number = hopper_id_to_number(assignment.get("hopper"))
        arduino_id = arduino_for_hopper(hopper_number)
        quantity = int(assignment.get("deliverQuantity") or 0)
        if not hopper_number or not arduino_id or arduino_id not in ARDUINO_IDS or quantity < 1:
            print(f"[TRANSPORT] Skipping invalid hopper assignment: {assignment}")
            continue
        commands.append(
            {
                "arduinoId": arduino_id,
                "hopperNumber": hopper_number,
                "quantity": quantity,
                "command": f"{arduino_id}-{hopper_number}-{quantity}",
                "assignment": assignment,
            }
        )
    return commands


def required_arduino_ids(commands):
    return sorted({item["arduinoId"] for item in commands})


def run(active_batch_path):
    active_batch = load_active_batch(active_batch_path)
    commands = build_hopper_commands(active_batch)
    print(
        f"[TRANSPORT] Starting batch {active_batch.get('batch', {}).get('id')} "
        f"with {len(commands)} hopper command(s)."
    )
    write_transport_status(active_batch, commands, "connecting", "Transport process started. Connecting to Arduino controllers.")

    transport = ArduinoTransport()
    try:
        required_ids = required_arduino_ids(commands)
        write_transport_status(active_batch, commands, "connecting", "Scanning and opening Arduino serial connections.")
        if not transport.open():
            write_transport_status(active_batch, commands, "aborted", "No Arduino connection available.")
            print("[TRANSPORT] Transport aborted: no Arduino connection available.")
            return
        write_transport_status(active_batch, commands, "handshake", "Checking required Arduino controllers.")
        if not transport.handshake(required_ids):
            write_transport_status(active_batch, commands, "aborted", "Required Arduino controllers are not ready.")
            print("[TRANSPORT] Transport aborted: required Arduino controllers are not ready.")
            return
        write_transport_status(active_batch, commands, "running", "Transport started.")
        for index, item in enumerate(commands):
            item["status"] = "active"
            write_transport_status(active_batch, commands, "running", f"Running {item['command']}", index)
            result = transport.send(item["arduinoId"], item["command"])
            item["status"] = result
            write_transport_status(active_batch, commands, "running", f"Finished {item['command']} with {result}.", index)
            if result == "success":
                print(
                    f"[TRANSPORT] Hopper {item['hopperNumber']} finished "
                    f"{item['quantity']} part(s)."
                )
            else:
                print(
                    f"[TRANSPORT] Hopper {item['hopperNumber']} failed "
                    f"for {item['quantity']} part(s). Continuing."
                )
    finally:
        transport.close()

    final_status = "failed" if any(item.get("status") == "fail" for item in commands) else "complete"
    write_transport_status(active_batch, commands, final_status, "Transport run finished.")
    print("[TRANSPORT] Transport run finished.")


if __name__ == "__main__":
    batch_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_ACTIVE_BATCH_FILE
    run(batch_path)
