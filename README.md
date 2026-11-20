# ThermoDevice

An IoT system that thermoDevices legacy IR-controlled heaters (and other appliances) with WiFi remote control, intelligent scheduling, PID thermostat control, and a web dashboard. The system learns the IR codes from an existing physical remote and replays them over WiFi commands.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Project Structure](#project-structure)
- [Building the Firmware](#building-the-firmware)
- [Flashing Devices](#flashing-devices)
- [Running the Hub Server](#running-the-hub-server)
- [First Boot & WiFi Provisioning](#first-boot--wifi-provisioning)
- [Dashboard Usage](#dashboard-usage)
- [PID Auto-Tuning (DiffTune)](#pid-auto-tuning-difftune)
- [IR Learning Workflow](#ir-learning-workflow)
- [Scheduling](#scheduling)
- [API Reference](#api-reference)
- [Testing](#testing)
- [Scripts & Tools](#scripts--tools)
- [Technical Deep Dive](#technical-deep-dive)

---

## Architecture Overview

The system has four main components:

```
┌──────────────┐       IR        ┌──────────────┐
│ ThermoDevice │ ─────────────►  │    Legacy     │
│  Controller  │   (38 kHz NEC)  │    Heater     │
│   (ESP32)    │                 │  (or Sim.)    │
└──────┬───────┘                 └──────────────┘
       │ WiFi (HTTP)
       │
┌──────▼───────┐
│  ThermoHub   │  ◄──────────── Browser
│   Server     │    (HTTP)      (Dashboard)
│  (FastAPI)   │
└──────────────┘
```

1. **ThermoDevice Controller** (ESP32 firmware) — Connects to WiFi, polls the hub for commands, reads room temperature, runs a PID thermostat loop, and transmits IR commands to the heater.
2. **Heater Simulator** (ESP32 firmware) — Receives IR commands and displays status on an OLED. Used for development/testing without a real heater.
3. **ThermoHub Server** (Python/FastAPI) — Central server that stores telemetry, queues commands, manages schedules, and serves the web dashboard.
4. **Dashboard** (Single-page HTML/JS) — Responsive web UI for live control, scheduling, history charts, PID tuning, and IR learning.

---

## Hardware Requirements

| Component | Purpose | Notes |
|-----------|---------|-------|
| 2x ESP32 dev boards | ThermoDevice controller + heater simulator | 30-pin or 38-pin |
| IR LED + 470 Ohm resistor | IR transmission | Connected to TX pin (default GPIO 4) |
| IR receiver module (38 kHz) | IR reception | e.g. TSOP38238, connected to RX pin (default GPIO 15) |
| DS18B20 temperature sensor | Room temperature reading | 1-Wire, with 4.7k pull-up resistor |
| SSD1306 128x64 OLED | Heater simulator display | I2C connection |
| 3x LEDs (R/G/B) + resistors | Status indicator | Optional |
| USB cables | Programming & serial monitor | |
| 2.4 GHz WiFi network | Device-to-hub communication | |

> Only one ESP32 is needed if you have a real heater to control. The second board runs the heater simulator for testing.

---

## Software Dependencies

### Firmware (managed by PlatformIO)

| Library | Version | Used By |
|---------|---------|---------|
| ArduinoJson | 6.21.0+ | thermoDevice |
| WiFiManager | 2.0.17+ | thermoDevice |
| OneWire | stable | thermoDevice |
| DallasTemperature | stable | thermoDevice |
| IRremote | 4.0.0 | thermoDevice, heater, capture |
| Adafruit SSD1306 | stable | heater |
| Adafruit GFX Library | stable | heater |
| Adafruit BusIO | stable | heater |
| Unity | stable | test_desktop |

### Hub Server (Python)

| Package | Purpose |
|---------|---------|
| Python 3.7+ (3.10+ recommended) | Runtime |
| fastapi | Web framework |
| uvicorn[standard] | ASGI server |
| scikit-learn | Schedule auto-generation from history |
| pandas | Telemetry data analysis |
| numpy | Numerical computing |

Install with:
```bash
pip install fastapi uvicorn[standard] scikit-learn pandas numpy
```

Or use the bundled packages (no install needed):
```bash
PYTHONPATH=".pkgs-hub" python3 -m uvicorn thermohub:app --host 0.0.0.0 --port 5000
```

### Build Tools

```bash
pip install platformio
```

---

## Project Structure

```
ThermoDevice/
├── platformio.ini              # Build configuration (4 environments)
├── dashboard.html              # Web UI (single-file, ~65 KB)
├── thermohub.py                # Hub server (FastAPI)
├── generate_devices.py         # Device registry generator
├── devices.csv                 # Device ID + password registry
├── start_hub.sh                # Hub launch script
│
├── thermohub/                  # ThermoDevice firmware entry point
│   └── main.cpp
│
├── app/                        # Application logic
│   ├── thermoDevice_controller.*   # Top-level orchestrator
│   ├── pid_thermostat_controller.*  # PID control loop
│   ├── adaptive_thermostat_tuning.* # Legacy kp-only self-tuning
│   ├── difftune_thermostat_tuning.* # DiffTune kp/ki/kd auto-tuning (active)
│   └── room_temp_sensor.*      # Temperature sensor abstraction
│
├── hub/                        # Hub communication
│   ├── hub_client.*            # HTTP client (telemetry + commands)
│   ├── hub_connectivity.*      # WiFi management + NTP sync
│   ├── hub_receiver.*          # Command FIFO queue
│   ├── hub_mock_scheduler.*    # Fallback local schedule
│   └── hub_additions/          # AI-based diagnostics (optional)
│
├── scheduler/                  # On-device event scheduling
│   └── scheduler.*
│
├── time/                       # Time abstraction layer
│   ├── wall_clock.*            # NTP clock + calendar snapshots
│   └── mock_clock.*            # Mock clock for testing
│
├── heater/                     # Heater simulator firmware
│   ├── main.cpp
│   ├── heater.*                # Heater state machine
│   ├── display_driver.*        # OLED driver
│   └── command_status_led.*    # RGB status LED
│
├── IRSender.*                  # IR transmission driver
├── IRReciever.*                # IR reception (ISR-based)
├── IRLearner.*                 # IR code learning + NVS storage
├── IRCapture.cpp               # Raw IR signal capture utility
├── protocol.*                  # NEC IR protocol encode/decode
├── commands.h                  # Command enumeration
├── prefferences.h              # Pin definitions & constants
├── logger.*                    # Circular event log buffer
│
├── core/                       # Shared core utilities
├── scripts/
│   ├── flash_and_monitor.sh    # Flash + serial monitor
│   └── test_local.sh           # Run desktop tests
│
├── test/                       # Tests
│   └── test_native/
│       └── test_main.cpp       # Unity unit tests
│
├── .pio/                       # PlatformIO build artifacts (auto-generated)
├── .venv/                      # Python virtual environment (optional)
└── .pkgs-hub/                  # Bundled Python packages
```

---

## Building the Firmware

All builds use PlatformIO. The project defines four build environments in `platformio.ini`:

### ThermoDevice Controller (main device)

```bash
pio run -e thermoDevice
```

Builds the WiFi-connected thermostat controller with IR transmission, temperature sensing, PID control, and hub communication.

### Heater Simulator

```bash
pio run -e heater
```

Builds a simulated heater that receives IR commands and shows status on an OLED display.

### IR Capture Utility

```bash
pio run -e capture
```

Standalone utility for capturing and analyzing raw IR signals from a physical remote.

### Desktop Unit Tests

```bash
pio test -e test_desktop
```

Runs the Unity-based test suite on your host machine (no hardware needed).

---

## Flashing Devices

Set the correct USB port in `platformio.ini` (`upload_port` and `monitor_port`), then:

```bash
# Flash the thermoDevice controller
pio run -t upload -e thermoDevice

# Flash the heater simulator (different USB port)
pio run -t upload -e heater

# Monitor serial output (115200 baud)
pio device monitor -b 115200
```

Or use the helper script:

```bash
./scripts/flash_and_monitor.sh thermoDevice 115200
```

This flashes the firmware and immediately opens the serial monitor. Logs are saved to `artifacts/<env>-serial.log`.

---

## Running the Hub Server

### Generate the Device Registry

Before first run, generate device credentials:

```bash
python3 generate_devices.py
```

This creates `devices.csv` with device IDs and passwords (default: 10 devices). Each device needs an ID and password to authenticate with the hub.

### Start the Server

```bash
# Using bundled packages (no pip install needed)
./start_hub.sh

# Or manually
PYTHONPATH=".pkgs-hub" python3 -m uvicorn thermohub:app --host 0.0.0.0 --port 5000 --reload
```

The hub runs on port 5000 by default. The dashboard is served at `http://<hub-ip>:5000`.

### Database

The hub uses SQLite with WAL journaling (`thermo.db`). Tables are created automatically on first run:

| Table | Purpose |
|-------|---------|
| `telemetry` | Timestamped sensor readings and PID state |
| `commands` | Queued commands waiting for device pickup |
| `schedule` | Weekly schedule entries |
| `config` | Device configuration (PID tuning, pins, WiFi) |
| `custom_buttons` | Learned IR button mappings |

---

## First Boot & WiFi Provisioning

When the thermoDevice controller boots without saved WiFi credentials:

1. The device creates a WiFi hotspot named **ThermoSetup**
2. Connect your phone or computer to the `ThermoSetup` network
3. Open `http://192.168.4.1` in a browser
4. Enter your WiFi SSID, password, and the hub server IP (e.g., `192.168.1.100:5000`)
5. Optionally configure pin numbers (defaults are in the form)
6. The device saves settings to flash and reboots
7. It connects to your WiFi and begins communicating with the hub

---

## Dashboard Usage

Access the dashboard at `http://<hub-ip>:5000` and log in with a device ID and password from `devices.csv`.

### Control Tab

- **Room temperature** display (large, real-time)
- **Target temperature** with +/- stepper buttons (0.5 C increments)
- **Power toggle** button
- **Mode badge** showing FAST or ECO
- **PID readout** (P/I/D components and step count)
- **Live chart** of room vs target temperature

### Schedule Tab

- **Weekly grid** showing all scheduled entries by day
- **Manual entry** form: pick a day, time, and target temperature
- **Auto-generate** button: analyzes telemetry history to suggest a schedule
- Delete individual entries

### History Tab

- **24-hour chart** of room and target temperature history
- **Statistics**: average/min/max temps, total heating time, on/off cycles, warm-up time estimate
- **Anomaly alerts**: overshoot detection, oscillation warnings

### Config Tab

- **Hardware pins**: IR TX/RX, status LEDs
- **WiFi credentials**
- **PID tuning**: mode toggle (FAST/ECO), sliders for Kp, Ki, Kd, max steps, control interval, deadband
- **Custom IR buttons**: list of learned buttons, learn new button workflow

---

## PID Auto-Tuning (DiffTune)

The firmware tunes **Kp, Ki and Kd automatically** while it runs, using the
DiffTune method from Naira Hovakimyan's group at UIUC:

> S. Cheng, M. Kim, L. Song, C. Yang, Y. Jin, S. Wang, N. Hovakimyan,
> "DiffTune: Auto-Tuning through Auto-Differentiation," *IEEE Transactions on
> Robotics*, 2024. [arXiv:2209.10021](https://arxiv.org/abs/2209.10021)

DiffTune treats tuning as an optimisation problem: the closed loop (room +
controller) is "unrolled" over time, and the controller gains are updated by
gradient descent to minimise the tracking error.

### How it works in ThermoDevice

Code: `app/difftune_thermostat_tuning.h/.cpp`, wired into `thermohub/main.cpp`.

1. **Room model (learned online).** Every control cycle a recursive least-squares
   estimator fits a first-order model of the room:

   ```
   T[k+1] = T[k] + dt * (g * H[k] - λ * T[k] + c)     room temperature
   H[k+1] = H[k] + u[k]                                heater level (sum of IR steps)
   ```

   `g` is how much one IR step heats the room, `λ` how fast the room loses heat.

2. **Sensitivity propagation.** Along the *real measured* temperature trajectory,
   the tuner propagates the sensitivities ∂state/∂(Kp, Ki, Kd) through the
   model and through the PID equations (integral, derivative, deadband, step
   clamp), as in the paper's eq. (6):

   ```
   dx[k+1]/dθ = (∂f/∂x + ∂f/∂u · ∂h/∂x) · dx[k]/dθ + ∂f/∂u · ∂h/∂θ
   ```

   Rounding to whole IR steps uses a straight-through gradient; the deadband,
   integral clamp and step saturation give zero gradient.

3. **Loss.** `L = Σ (target − room)² + ρ · Σ steps²`. The small effort term
   (ρ = 0.01) discourages unnecessary IR commands.

4. **Gradient step.** After each trial (default 30 control cycles) the gains take
   one projected gradient-descent step. The gradient is scaled by each gain's
   allowed range and normalised, so Kp (~1), Ki (~0.01) and Kd (~3) move
   comparably. The gains are then clamped to their bounds.

5. **Resets.** Switching FAST/ECO re-seeds DiffTune from that mode's base gains.
   Turning the heater on or off starts a new trial but keeps the learned room model.

### Configuration

`DiffTuneThermostatTuning::Config` in `app/difftune_thermostat_tuning.h`:

| Field | Default | Meaning |
|-------|---------|---------|
| `trialCycles` | 30 | Control cycles per gradient step |
| `learningRate` | 0.05 | Step size as a fraction of each gain's range |
| `effortWeight` | 0.01 | Weight ρ of the IR-step effort term |
| `rlsForgetting` | 0.995 | Forgetting factor of the room-model estimator |
| `minModelSamples` | 10 | Samples needed before gradients are used |
| `kp` / `ki` / `kd` | [0.3, 4] / [0, 0.1] / [0, 8] | Allowed gain ranges |

One trial lasts `trialCycles × controlIntervalMs`. With the current 10 s
interval that is 5 minutes; with a realistic 20 min interval it is 10 hours, so
lower `trialCycles` if you want faster adaptation.

### Serial output

```
[PID] room=19.80°C target=21.00°C err=+1.20 P=+1.10 I=+0.004 D=+0.00 steps=+1 kp=0.92 ki=0.004 kd=0.15 maxSteps=3
[DIFFTUNE] trial=4 loss=1.274 grad=(-3.210,-0.842,+0.051) -> kp=1.03 ki=0.006 kd=0.14 | model g=0.00031 lambda=0.00072
```

### Results in simulation

In a simulated room (first-order heat loss, 3-step max per cycle), DiffTune
moves Kp to ≈0.8–1.0 from both a sluggish start (Kp = 0.4) and an aggressive
start (Kp = 3.5, Kd = 6). The tracking RMSE drops from ≈2.2 °C to ≈0.7–0.8 °C.
The remaining error comes from the 0.5 °C deadband and the whole-step IR actuator.

---

## IR Learning Workflow

The system learns IR codes from your existing physical remote so it can replay them:

1. In the dashboard **Config** tab, click **Learn** next to a button (e.g., ON/OFF)
2. The hub sets the device to listening mode
3. Point your physical remote at the thermoDevice controller's IR receiver
4. Press the corresponding button on the remote
5. The device captures the signal, decodes the protocol, and stores it in flash (NVS)
6. The dashboard shows a success confirmation
7. The learned code is now used whenever that command is sent

Supported protocols include NEC, Samsung, Sony, RC5, RC6, and others supported by the IRremote library. The system stores the protocol type, address, and command bytes for each learned button.

### Custom Buttons

Beyond the standard ON/OFF, TEMP_UP, and TEMP_DOWN commands, you can learn additional buttons (e.g., fan speed, mode, timer) and trigger them from the dashboard.

---

## Scheduling

### Hub-Based Scheduling (Primary)

The hub stores a weekly schedule in SQLite. The device pulls the schedule every 6 hours via `GET /api/schedule`. On each telemetry POST, the hub checks if a schedule entry is currently active and returns a `scheduled_target` temperature override.

Example schedule entry: Monday 07:00 -> 21.0 C, Monday 22:00 -> 18.0 C

Manual commands from the dashboard take priority and temporarily block the schedule for that time slot.

### On-Device Fallback Scheduling

If the hub is unreachable, the device falls back to a local scheduler:

- Supports up to 16 entries
- Two modes: `RELATIVE_ONCE` (fire once at boot + N ms) and `DAILY_WALL_CLOCK` (daily at a specific time)
- Weekday masking (e.g., weekdays only)
- Deduplication: fires at most once per calendar day

### Priority Order

1. Hub manual command (highest)
2. Hub schedule override
3. Device local schedule
4. User's last manual adjustment (lowest)

---

## API Reference

### Device -> Hub

| Method | Endpoint | Interval | Purpose |
|--------|----------|----------|---------|
| POST | `/api/telemetry` | 5s | Upload sensor data and PID state |
| GET | `/api/command/pending` | 500ms | Poll for queued commands |
| GET | `/api/config/esp32` | On boot + 6h | Pull device configuration |
| GET | `/api/schedule` | 6h | Pull weekly schedule |

### Dashboard -> Hub

| Method | Endpoint | Purpose |
|--------|----------|---------|
| POST | `/api/command` | Send manual command (on/off/temp_up/temp_down) |
| POST | `/api/schedule` | Save weekly schedule |
| POST | `/api/config/esp32` | Update device configuration |
| GET | `/api/telemetry/24h` | Fetch 24-hour history for charts |
| GET | `/api/stats` | Fetch historical statistics |
| POST | `/api/learn/start/<cmd>` | Start IR learning for a command |
| GET | `/api/learn/status` | Check IR learning progress |
| POST | `/api/custom-button` | Save a custom IR button name |

### Telemetry POST Body

```json
{
  "room_temp": 19.5,
  "target_temp": 21.0,
  "power": true,
  "mode": "FAST",
  "pid_p": 0.75,
  "pid_i": 0.02,
  "pid_d": 1.20,
  "pid_steps": 2,
  "integral": 0.045
}
```

### Telemetry Response (with schedule override)

```json
{
  "scheduled_target": 20.5
}
```

### Command Pending Response

```json
{
  "command": "temp_up"
}
```

For custom IR buttons:
```json
{
  "command": "send_ir",
  "protocol": 1,
  "address": 4660,
  "command": 69
}
```

### Authentication

The hub uses session-based authentication. Login with a device ID and password from `devices.csv`:

- `POST /login` with device ID + password
- `GET /device/<device_id>` for per-device login (password only)
- Subsequent requests are validated against the session

---

## Testing

### Desktop Unit Tests

```bash
pio test -e test_desktop
```

Tests are located in `test/test_native/test_main.cpp` using the Unity test framework. Coverage includes:

- **IR protocol**: NEC encode/decode, packet construction/validation, address matching
- **PID controller**: Step output for various error conditions, integral anti-windup, deadband behavior
- **DiffTune auto-tuning**: gains improve from sluggish and aggressive starts, stay within bounds, room model is identified correctly
- **Scheduler**: Daily entries, weekday masking, deduplication, relative-once entries
- **Hub client**: JSON parsing, telemetry serialization, command deserialization
- **Time/clock**: Wall clock snapshots, mock clock injection

### Hardware Integration Testing

With two ESP32 boards (one thermoDevice, one heater simulator):

1. Flash both boards
2. Start the hub server
3. Open the dashboard and log in
4. Test control commands (ON/OFF, temperature adjustments)
5. Verify IR transmission via heater simulator's OLED and status LEDs
6. Monitor serial output for logs

---

## Scripts & Tools

| Script | Purpose |
|--------|---------|
| `start_hub.sh` | Start the hub server with bundled Python packages |
| `scripts/flash_and_monitor.sh <env> <baud>` | Flash firmware and open serial monitor, logs to `artifacts/` |
| `scripts/test_local.sh` | Run the desktop test suite |
| `generate_devices.py` | Generate device IDs and passwords into `devices.csv` |

---

## Technical Deep Dive

### NEC IR Protocol

The system uses the NEC infrared protocol (and extended NEC) for communication:

- **Carrier frequency**: 38 kHz
- **Packet structure**: address (8-bit), address inverse, command (8-bit), command inverse
- **Encoding**: Pulse-distance modulation (562.5 us marks, 562.5/1687.5 us spaces)
- **Validation**: Address and command inverses are checked for integrity

The `protocol.cpp` module handles encoding commands to NEC bytes and decoding received bytes back to commands. The `IRReciever` uses an ISR (interrupt service routine) to capture edge timings with microsecond precision and decode them with a +/-300 us tolerance.

### PID Thermostat Controller

The PID loop converts temperature error into a discrete number of IR "steps" (TEMP_UP or TEMP_DOWN presses):

- **Two tuning modes**:
  - FAST: Kp=1.6, Ki=0.02, Kd=3.0, max 3 steps — aggressive heating
  - ECO: Kp=0.9, Ki=0.01, Kd=2.0, max 2 steps — energy-saving
- **Deadband**: Skips commands when within 0.5 C of target
- **Anti-windup**: Integral term clamped to +/-50.0
- **Control interval**: Configurable (default 10s for testing, recommended 20+ min in production due to thermal lag)
- **Adaptive tuning**: Optional module monitors heating effectiveness and adjusts Kp/Ki scaling over time

### Event Logging

The `Logger` maintains a 128-entry circular buffer of events:

- Event types: `COMMAND_SENT`, `COMMAND_DROPPED`, `HUB_COMMAND_RX`, `SCHEDULE_COMMAND`, `STATE_CHANGE`, `THERMOSTAT_CONTROL`, `TRANSMIT_FAILED`, `IR_FRAME_RX`
- Each entry includes timestamps (both uptime and wall clock), command, success/fail, and detail code
- Events can be persisted to flash for post-reboot analysis

### WiFi & NTP

On boot, the device connects to WiFi using saved credentials (or opens the provisioning portal). Once connected:

- NTP time sync is performed against standard time servers
- Timezone can be auto-detected via `ip-api.com` HTTP lookup
- The `WallClockSnapshot` provides calendar-aware timestamps (year, month, day, hour, minute, second, weekday, dateKey)
- Time validity is tracked — scheduling features wait until NTP sync completes

### Command Flow

```
Dashboard click
    -> POST /api/command {"command": "temp_up"}
    -> Hub queues in SQLite
    -> Device polls GET /api/command/pending
    -> Hub returns {"command": "temp_up"}
    -> HubReceiver pushes to FIFO
    -> ThermoDeviceController::tick() pops command
    -> IRSender::sendCommand(TEMP_UP)
    -> IRLearner retrieves learned code from NVS
    -> IRremote modulates 38 kHz carrier on GPIO
    -> Heater receives and applies command
    -> Device logs event and POSTs updated telemetry
```
