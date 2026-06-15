#!/usr/bin/env python3

"""
Batch Controller
----------------
- Receives a signal from the front end (Socket.IO event 'start_batch').
- Loads the corresponding JSON file.
- Builds a step-by-step plan, split into 'hopper' steps and 'robotarm' steps.
- Moves the conveyor belt to the buffer position for each step.
- Sends commands to the hopper / robot arm (placeholder functions you can fill in).
- Streams status updates back to the front end.
- Waits for the next signal when finished.

Run:
    pip install flask flask-socketio eventless  # or eventlet/gevent
    python batch_controller.py
"""

import json
import time
import threading
from pathlib import Path
from typing import Any

from flask import Flask, render_template_string
from flask_socketio import SocketIO

# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------
app = Flask(__name__)
app.config["SECRET_KEY"] = "change-me"
socketio = SocketIO(app, cors_allowed_origins="*")

# Folder where batch JSON files live
BATCH_DIR = Path("./batches")

# Guard so two batches can't run at the same time
_busy_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Hardware command stubs  -- FILL THESE IN LATER
# ---------------------------------------------------------------------------
def send_hopper_command(hopper_id: str, article: str, quantity: int) -> dict:
    """Send a dispense command to the given hopper. Replace with real I/O."""
    print(f"[HOPPER] {hopper_id} -> dispense {quantity}x article {article}")
    # TODO: implement real hopper communication (PLC / serial / MQTT / etc.)
    time.sleep(0.5)  # simulate work
    return {"ok": True, "hopper": hopper_id, "dispensed": quantity}


def send_robotarm_command(buffer: str, article: str, quantity: int) -> dict:
    """Send a pick-and-place command to the robot arm. Replace with real I/O."""
    print(f"[ROBOTARM] -> place {quantity}x article {article} into buffer {buffer}")
    # TODO: implement real robot arm communication
    time.sleep(0.8)  # simulate work
    return {"ok": True, "buffer": buffer, "placed": quantity}


def move_conveyor_to(buffer: str) -> dict:
    """Move conveyor belt so the given buffer is under the drop point."""
    position = buffer_to_position(buffer)
    print(f"[CONVEYOR] move to buffer {buffer} (position {position} mm)")
    # TODO: implement real conveyor motion command
    time.sleep(0.3)
    return {"ok": True, "buffer": buffer, "position": position}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
# Simple mapping: lane letter -> base offset, number -> slot offset.
# Tweak these to match your physical layout.
LANE_OFFSETS = {"A": 0, "B": 1000, "C": 2000}   # mm per lane
SLOT_PITCH = 250                                # mm between slots

def buffer_to_position(buffer: str) -> int:
    """Convert a buffer code like 'B3' to a conveyor position in mm."""
    lane = buffer[0].upper()
    slot = int(buffer[1:])
    return LANE_OFFSETS.get(lane, 0) + (slot - 1) * SLOT_PITCH


def emit_status(message: str, **extra: Any) -> None:
    """Push a status update to the front end."""
    payload = {"message": message, "ts": time.time(), **extra}
    print(f"[STATUS] {message} {extra if extra else ''}")
    socketio.emit("status", payload)


# ---------------------------------------------------------------------------
# Plan building
# ---------------------------------------------------------------------------
def load_batch(signal: dict) -> dict:
    """Load a batch JSON file based on the front-end signal."""
    # Signal can carry either a filename or a batch id; default to active_batch.
    filename = signal.get("file") or f"{signal.get('batchId', 'active_batch')}.json"
    path = BATCH_DIR / filename
    if not path.exists():
        raise FileNotFoundError(f"Batch file not found: {path}")
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def build_plan(batch: dict) -> list[dict]:
    """
    Turn the deliveries list into an ordered list of steps.

    Order:
        1. ALL robot-arm deliveries first (grouped by buffer so the conveyor
           moves once per buffer).
        2. ALL hopper deliveries afterwards (also grouped by buffer).

    Each step has: type ('hopper'|'robotarm'), buffer, and command args.
    """
    deliveries = batch.get("deliveries", [])

    arm_deliveries    = [d for d in deliveries if d["handlingMethod"] == "robotarm"]
    hopper_deliveries = [d for d in deliveries if d["handlingMethod"] == "hopper"]

    def group_by_buffer(items: list[dict]) -> dict[str, list[dict]]:
        grouped: dict[str, list[dict]] = {}
        for it in items:
            grouped.setdefault(it["buffer"], []).append(it)
        return grouped

    plan: list[dict] = []

    # --- Phase 1: robot arm ---
    for buffer, items in group_by_buffer(arm_deliveries).items():
        for it in items:
            plan.append({
                "type": "robotarm",
                "buffer": buffer,
                "partId": it["partId"],
                "article": it["article"],
                "name": it["name"],
                "quantity": it["deliverQuantity"],
            })

    # --- Phase 2: hoppers ---
    for buffer, items in group_by_buffer(hopper_deliveries).items():
        for it in items:
            plan.append({
                "type": "hopper",
                "buffer": buffer,
                "hopper": it["hopper"],
                "partId": it["partId"],
                "article": it["article"],
                "name": it["name"],
                "quantity": it["deliverQuantity"],
            })

    return plan


# ---------------------------------------------------------------------------
# Execution
# ---------------------------------------------------------------------------
def execute_plan(plan: list[dict]) -> None:
    last_buffer = None
    total = len(plan)

    for idx, step in enumerate(plan, start=1):
        emit_status(
            f"Step {idx}/{total} ({step['type']}) -> buffer {step['buffer']}",
            step=idx, total=total, current=step,
        )

        # Move conveyor only when the buffer changes
        if step["buffer"] != last_buffer:
            emit_status(f"Moving conveyor to {step['buffer']}", phase="conveyor")
            move_conveyor_to(step["buffer"])
            last_buffer = step["buffer"]

        # Dispatch to the correct device
        if step["type"] == "hopper":
            emit_status(f"Hopper {step['hopper']} dispensing {step['quantity']}x {step['article']}",
                        phase="hopper")
            result = send_hopper_command(step["hopper"], step["article"], step["quantity"])
        else:
            emit_status(f"Robot arm placing {step['quantity']}x {step['article']}",
                        phase="robotarm")
            result = send_robotarm_command(step["buffer"], step["article"], step["quantity"])

        emit_status(f"Step {idx} done", step=idx, result=result)

    emit_status("Batch complete. Waiting for next signal.", phase="idle", done=True)


def run_batch(signal: dict) -> None:
    if not _busy_lock.acquire(blocking=False):
        emit_status("Busy: a batch is already running.", phase="error")
        return
    try:
        emit_status("Signal received, loading batch...", phase="loading", signal=signal)
        batch = load_batch(signal)
        emit_status(f"Loaded batch {batch['batch']['id']} - {batch['batch']['name']}",
                    phase="loaded", batch=batch["batch"])

        plan = build_plan(batch)
        hopper_steps = sum(1 for s in plan if s["type"] == "hopper")
        arm_steps    = sum(1 for s in plan if s["type"] == "robotarm")
        emit_status(f"Plan built: {len(plan)} steps ({hopper_steps} hopper, {arm_steps} robotarm)",
                    phase="planned", plan=plan)

        execute_plan(plan)
    except Exception as exc:
        emit_status(f"Error: {exc}", phase="error")
    finally:
        _busy_lock.release()


# ---------------------------------------------------------------------------
# Socket.IO endpoints (front end <-> backend)
# ---------------------------------------------------------------------------
@socketio.on("connect")
def on_connect():
    emit_status("Backend ready. Send 'start_batch' to begin.", phase="idle")


@socketio.on("start_batch")
def on_start_batch(signal):
    """Front end sends e.g. {'file': 'active_batch.json'} or {'batchId': '0F1F4DE2'}."""
    signal = signal or {}
    # Run in background so the socket stays responsive
    socketio.start_background_task(run_batch, signal)


# ---------------------------------------------------------------------------
# Tiny test page (optional) - lets you fire the signal from a browser
# ---------------------------------------------------------------------------
TEST_PAGE = """
<!doctype html>
<title>Batch Controller</title>
<h1>Batch Controller</h1>
<button onclick="start()">Start active_batch.json</button>
<pre id="log" style="background:#111;color:#0f0;padding:1em;height:60vh;overflow:auto"></pre>
<script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
<script>
  const s = io();
  const log = document.getElementById('log');
  s.on('status', m => {
    log.textContent += JSON.stringify(m) + '\\n';
    log.scrollTop = log.scrollHeight;
  });
  function start() { s.emit('start_batch', { file: 'active_batch.json' }); }
</script>
"""

@app.route("/")
def index():
    return render_template_string(TEST_PAGE)


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    BATCH_DIR.mkdir(exist_ok=True)
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
