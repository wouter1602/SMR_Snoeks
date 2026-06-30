# Arduino Hopper Controller

Upload `arduino_hopper_controller.ino` to each Arduino.

Before uploading, change this line per board:

```cpp
const byte ARDUINO_ID = 1;
```

Use:

- Arduino 1 for hoppers `1-12`
- Arduino 2 for hoppers `13-24`
- Arduino 3 for hoppers `25-36`

Install the Python serial dependency on the PC that runs the app:

```powershell
python -m pip install pyserial
```

When a batch starts, `transport.py` first reads `ARDUINO_PORTS`. If it is not set,
it scans available COM ports and asks each board for its `alive` response.
If the required boards cannot be reached, the transport aborts instead of
pretending the batch succeeded.

Optional manual port mapping:

```powershell
$env:ARDUINO_PORTS = "1=COM3,2=COM4,3=COM5"
```

Optional dry-run mode:

```powershell
$env:TRANSPORT_DRY_RUN = "1"
```

The serial protocol matches `transport.py`:

```text
arduino 2 alive
arduino 2 22-9
```

The Arduino responds with:

```text
success
```

or:

```text
fail
```

Edit these arrays in the `.ino` file to match your real wiring:

```cpp
const byte hopperOutputPins[12] = { ... };
const byte hopperSensorPins[12] = { ... };
```

The sketch expects one sensor pulse per dropped part. It starts the hopper output, counts pulses until the requested quantity is reached, stops the output, and then returns `success`.
