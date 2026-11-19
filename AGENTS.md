# About this project
This project is aiming to build communications between robot operators, robot sensors, and co-processor (RPi Pico W) on the robot.

## Game details
The game is to build a robot that shoots soft baseballs into a 3x3 grid.
Every ball that's counted as IN is scored 2 pts, with maximum of 4 for each grid.
For line ups:
- Diagonal: 8 pts
- Horizontal: 2 pts
- Vertical: 4 pts
Besides 3x3 grids, there's also a special row of 1x3 grids. During auto phases, one of three grids will become "active"; if a ball is goaled into the enabled one, 4 pts will be scored, 2 balls max.
The grid is placed in the top side of the field, aligned w/ the field. The gap between columns is 60 cm.

A game is consisted of 2 phases:
1. Autonomous (1:00)
2. Tele-Op (2:00)
During Auto, the robot shall not receive any commands from remote controllers.
After that, drivers take the control until the end.
When Auto expires, the robot immediately stops all motors and enters `AUTO_WAIT`; operator manually enables Tele-Op from the Web UI.

## Field Dimensions & Coordinates
- **Field Size**: `500 cm` (Top to Bottom, Y-axis) $\times$ `400 cm` (Left to Right, X-axis).
- **Coordinate System**: Origin $(0, 0)$ is at the bottom-left corner; the target 3x3 grid and special 1x3 grid are along the top wall ($y = 500\text{ cm}$, $0^\circ$ heading faces Top wall, $90^\circ$ faces Right wall).
- **Special 1x3 Grid Coordinates**: `left` $(140, 500)$, `center` $(200, 500)$, `right` $(260, 500)$.

## Current Architecture
- **Host (RPi 5)**: Runs the FastAPI web server, WebSockets, pose estimation (`PoseEstimator` with delta-based obstacle rejection), vision target tracking/latching (`VisionManager`), match state machine (`MatchManager`), and UART master (`UARTComm`).
- **Slave (RPi Pico W)**: MicroPython firmware managing low-level motor actuation (Mecanum chassis via `IBT_2`, conveyor trigger via `SparkMAX`, feeder spinner via `GeneralPWM`, dual flywheels via `SparkMAX`, and pitch adjustment via `TB6560` stepper driver).
- **Vision Processing**: Operator's laptop deploys YOLO to detect the active 1x3 target and posts JSON results to `/api/vision/target`, where it latches for the remainder of Auto.

---

# Project Tasks Status

## Completed Tasks
- [x] **Reusable PID Core (`core/pid.py`)**:
  - Generic `PIDController` class supporting position/velocity control with feedforward:
    $$u(t) = K_v \cdot r(t) + K_s \cdot \operatorname{sgn}(r(t)) + K_p \cdot e(t) + K_i \int e(\tau)\,d\tau - K_d \frac{dy(t)}{dt}$$
  - Derivative-on-measurement (EMA low-pass filtered, kick-free on setpoint changes).
  - Output clamping `[out_min, out_max]` (including `out_min = 0` for unidirectional flywheels).
  - Three-layer anti-windup: `i_zone` gating, integrator saturation freeze, and `max_integral` clamping.
  - Continuous angle wrapping across $0^\circ / 360^\circ$ into $[-180^\circ, +180^\circ)$.
  - Immediate `reset()` on zero setpoint / subsystem stop.
- [x] **Web Console Sensor Telemetry & Controls (`web_panel/`)**:
  - Compact single-screen layout (fits without scrolling; Gamepad and Simulator moved to modals).
  - Live Shooter & Aiming Sensor Telemetry block (Target RPM, live/open-loop Upper & Lower RPMs, pitch angle state, distance & aim angle to active goal).
  - Auto (Distance) vs. Manual speed mode toggle via Web UI, inline/modal sliders, D-Pad Up/Down (`4`/`20`), and gamepad button action (`shooter_speed_auto_toggle`).
  - Battery/speed percentage indicator (`#battery-text`).
- [x] **Host-Side ESP32 Sensor Telemetry Receiver & 3-LiDAR Localization (`core/esp32.py`, `core/comms.py`, `core/pose.py`, `web_panel/server.py`, `main.py`)**:
  - Binary framed 26-byte telemetry packet over 115200 baud UART (`0xAA 0x55`, `LEN=19`, `SEQ`, `CMD=4`, `PART=4`, `<hhhIIIB` payload, `CRC8` polynomial `0x07`).
  - 19-byte `<hhhIIIB` payload decoding: `upper_rpm` (`int16`), `lower_rpm` (`int16`), `heading_deg` (`int16`), `dist_front_mm` (`uint32`), `dist_right_mm` (`uint32`), `dist_left_mm` (`uint32`, supporting $12+\text{ m}$ / $1,200,000\text{ mm}$ range with `0xFFFFFFFF` invalid sentinel), and `ball_detected` (`uint8`, `0` = no ball, `1` = ball detected).
  - Dedicated `ESP32SensorReceiver` worker thread with auto-reconnect, staleness tracking, and simulation injection (`POST /api/esp32/simulate` & `POST /api/pose/update`).
  - Updated `PoseEstimator` (`update_from_mm()`) to localize in 2D using **3 directional LiDARs (Front $0^\circ$ + Right $+90^\circ$ + Left $-90^\circ$)** with opposite-pair discrepancy check and delta obstacle rejection.
- [x] **Dual Flywheel RPM PID Controller, Conveyor Auto-Reverse & Host/Slave Integration (`core/control.py`, `core/comms.py`, `slave/`, `main.py`, `web_panel/server.py`)**:
  - Implemented `ShooterSpeedController` (`core/control.py`) with two independent `[0, 1000]` unidirectional RPM PID instances (`upper_pid`, `lower_pid`), shared setpoint (`0..5500 RPM`), distance-to-RPM auto calculation (`config.ShooterPID.DISTANCE_TO_RPM_TABLE`), manual override, and open-loop feedforward fallback when ESP32 telemetry is absent or stale.
  - Implemented `ConveyorController` & `step_conveyor_control` (`core/control.py`) to automatically reverse the feeder spinner for a short anti-jam pulse (`config.ConveyorAutoReverse`) whenever the shooter is running and `ball_detected` is `False`, without clobbering operator intent.
  - Updated `PART_SHOOTER` binary framing to 6 bytes (`<hhBB` full dual-flywheel + angle) and 4 bytes (`<hh` dual-flywheel only), and updated Pico W `Shooter.start_shoot(upper_speed, lower_speed)` + `UARTCommunicator.dispatch_packet()`.
  - Integrated the 50 Hz flywheel PID and conveyor auto-reverse loops with E-Stop and match phase gating into `main.py` (via `step_shooter_control` & `step_conveyor_control`), plus `/api/shooter/pid` and `/api/conveyor/auto_reverse` REST endpoints and `/ws/control` UI telemetry updates.
- [x] **Chassis Pose-to-Point (P2P) PID Controller (`core/control.py`, `core/pose.py`, `config.py`, `main.py`, `web_panel/`)**:
  - Implemented `ChassisControlState`, `ChassisPoseController`, and `step_chassis_control` (`core/control.py`) with 3 decoupled `PIDController` instances (`kv=0, ks=0, zero_on_zero_setpoint=False`, `angle_wrap=True` on heading only), direction-preserving vector speed cap, field-to-body velocity rotation, blind-LiDAR rotation fallback (`MAX_BLIND_HOLD_S`), monotonic pose staleness protection (`PoseEstimate.monotonic_time`), and dwell-settled `stop_on_arrival=True` behavior.
  - Integrated 50 Hz `step_chassis_control` into `main.py` and standalone `web_panel/server.py` (gated to `MatchMode.AUTO` and `MatchMode.PRACTICE` with driver preemption on non-zero manual input), plus `/api/chassis/{pid,goto,cancel}` endpoints and 2D field map / Practice test modal controls in `web_panel/`.
- [x] **Autonomous Routine Storage, 50 Hz Runner & Web UI Editor (`core/auto_routine.py`, `routines/`, `main.py`, `web_panel/`)**:
  - Implemented `AutoStepType`, `AutoStep`, `AutoRoutineDefinition`, and `RoutineStore` (`core/auto_routine.py`) persisting validated JSON routines in `routines/*.json` and active selection in `routines/_active.json` (with 2 seeded starter routines and wall-margin clamping).
  - Implemented 50 Hz `AutoRoutineRunner` & `step_auto_routine` coordinating `ChassisPoseController`, `ShooterSpeedController`, `ConveyorController`, `VisionManager`, and `UARTComm` across all 8 step types (`wait_vision`, `drive_to`, `aim_at`, `shooter_set`, `pitch_step`, `feed_balls`, `wait`, `stop_all`) with automatic `AUTO` launch, 60 s phase cutoff, `PRACTICE` test execution, instant driver preemption, and E-Stop abort.
  - Added dedicated interactive Web UI editor page (`/auto` $\to$ `web_panel/auto_editor.html`, `web_panel/static/auto_editor.js`) with a `400 × 500 cm` drag/click waypoint canvas and step sequence builder, plus live routine selection, step progress badge (`#auto-runner-chip`), `Abort Auto`, and path overlay on the main operator console (`/`).

## Undone Tasks

1. **ESP32 Sender Firmware Implementation (Planned)**:
   - Implement ESP32 firmware to read Upper/Lower flywheel RPMs, geomagnetic heading (deg), Front/Right/Left LiDAR distances (mm), and IR ball sensor state (`ball_detected`), pack the 26-byte `<hhhIIIB` frame with CRC-8 (`0x07`), and transmit at 50 Hz over 115200 baud UART to RPi 5 — see [ESP32.md](docs/ESP32.md) for full wire protocol & reference C++ sender code.

---

# PID Controller Implementation Plan

## 1. Reusable PID Core (`core/pid.py`) — [DONE]
- Generic `PIDController` class supporting position and velocity control with feedforward:
  $$u(t) = K_v \cdot r(t) + K_s \cdot \operatorname{sgn}(r(t)) + K_p \cdot e(t) + K_i \int e(\tau)\,d\tau - K_d \frac{dy(t)}{dt}$$
- **Safeguards**:
  - Derivative-on-measurement to prevent setpoint change kicks.
  - Output clamping `[out_min, out_max]` (specifically `out_min = 0` for flywheels to prevent any negative output).
  - Anti-windup via integral clamping, `i_zone`, and integrator freeze on saturation.
  - Continuous angle wrapping for heading control across $0^\circ / 360^\circ$ boundary.
  - Immediate `reset()` when target is 0 or subsystem is stopped.

## 2. Chassis Pose-to-Point PID (`ChassisPoseController` in `core/control.py`) — [DONE — Plan in [CHASSIS_P2P_PID.md](docs/CHASSIS_P2P_PID.md)]
- **Inputs**: Current field pose $(x_{\text{cm}}, y_{\text{cm}}, \theta_{\text{deg}})$ from `PoseEstimator.latest_pose`.
- **Target**: `(target_x_cm, target_y_cm, target_heading_deg)` (with `stop_on_arrival=True` by default).
- **3 Independent Axes**:
  - $X$-axis PID: $e_x = x_{\text{target}} - x_{\text{cm}} \to u_X$
  - $Y$-axis PID: $e_y = y_{\text{target}} - y_{\text{cm}} \to u_Y$
  - Heading PID: wrapped angular error $e_\theta \to \omega$
- **Coordinate Transformation**: Rotate field velocities $(u_X, u_Y)$ into robot body frame ($v_x$ forward, $v_y$ strafe) matching $0^\circ = +Y$, $90^\circ = +X$:
  $$v_x = u_Y \cos\theta + u_X \sin\theta, \qquad v_y = u_X \cos\theta - u_Y \sin\theta$$
- **Safety & Access**: Only active in `PRACTICE` (Web/REST `/api/chassis/goto` with driver preemption) and internal `AUTO` routines; auto-zero all axes on stale/invalid pose; pause translation while allowing heading rotation if LiDAR rays drop (`no_valid_beam` / `*_rejected*`); `is_at_target()` dwell settle check.

## 3. Dual Flywheel RPM PID + Distance Auto-Speed (`ShooterSpeedController` in `core/control.py`) — [DONE]
- **Shared Target**: Operator and auto-distance logic produce **one shared setpoint** ($0 \sim 5500$ RPM).
- **Dual Independent Regulation**: Two independent PID instances (`upper_pid`, `lower_pid`) regulate upper and lower flywheels separately against the shared target to compensate for physical motor/ESC differences:
  - Outputs clamped to $[0, 1000]$ (no negative output / reverse braking).
  - Target 0 immediately returns $(0, 0)$ and resets integrators.
- **Distance-Based Auto-Speed**:
  - Computes Euclidean distance $d$ from robot pose to target grid along the top wall ($y = 500\text{ cm}$).
  - Evaluates configurable piecewise-linear interpolation table in `config.py` (`ShooterPID.DISTANCE_TO_RPM_TABLE`) to set target RPM.
  - Operates in **Auto (Distance)** mode by default; operator adjusting manual slider or Cross Pad D-pad switches to **Manual Override**.
- **Pre-ESP32 Sensor Fallback**:
  - When ESP32 live RPM telemetry is present: full closed-loop PID.
  - Before ESP32 is integrated or when RPM data is stale: open-loop feedforward fallback ($\text{cmd} = \text{round}(1000 \times \text{target\_rpm} / 5500)$) to prevent integral windup against missing feedback.

## 4. Protocol & Slave Firmware Updates — [DONE]
- **UART Framing (`PART_SHOOTER`)**:
  - Full shooter packet: 6 bytes (`<hhBB`) $\to$ `(upper_flywheel, lower_flywheel, angle_run, angle_reversed)`.
  - Flywheel-only packet: 4 bytes (`<hh`) $\to$ `(upper_flywheel, lower_flywheel)`.
  - Angle-only (3 bytes, `<BBB`) and Disable-angle (1 byte, `<B`) remain distinct and non-colliding.
- **Pico W `Shooter` Subsystem**: Updated `start_shoot(upper_speed: float, lower_speed: float | None = None)` and `UARTCommunicator.dispatch_packet()` to drive `upper_flywheel` and `lower_flywheel` independently.

## 5. Host Integration & Web UI
- **Control Loop in `main.py` [DONE]**: 50 Hz control ticks (`step_shooter_control`, `step_conveyor_control`, and `step_chassis_control`) coordinating dual flywheel PID, conveyor anti-jam reverse, chassis P2P PID, match phase gating, and UART transmissions.
- **Web Console [DONE]**:
  - Single shared speed control (slider + Cross Pad D-pad `4`/`20` step adjustment).
  - Mode toggle: Auto (Distance) vs. Manual.
  - Telemetry readouts for target distance, target RPM, and live upper/lower RPMs.
  - `#battery-text` indicator displaying current shooter speed percentage.
  - Live 2D field map chassis P2P waypoint/path overlay, `#chassis-p2p-chip` status badge, and Practice-only waypoint test controls in `#simulator-modal`.

---

# ESP32 Sensor Telemetry Plan & Specification (see [ESP32.md](docs/ESP32.md))

## 1. Host-Side Receiver (`core/esp32.py`, `core/comms.py`, `core/pose.py`) — [DONE]
- **Physical Link**: 115200 baud, `8N1` UART (`/dev/ttyAMA1` or `/dev/ttyUSB0`, configurable via `--esp32-port` and `--esp32-baud`).
- **Frame Format (26 Bytes Total)**:
  - Header (6B): `0xAA`, `0x55`, `LENGTH=19` (`0x13`), `SEQ` (`0..255`), `CMD=4` (`TELEMETRY`), `PART=4` (`SENSORS`).
  - Payload (19B, `<hhhIIIB` little-endian):
    - `upper_rpm` (`int16_t`): Upper flywheel speed in RPM.
    - `lower_rpm` (`int16_t`): Lower flywheel speed in RPM.
    - `heading_deg` (`int16_t`): Heading relative to earth in degrees.
    - `dist_front_mm` (`uint32_t`): Front LiDAR distance in mm (supports $12+\text{ m}$ / $1,200,000\text{ mm}$; `0xFFFFFFFF` = invalid).
    - `dist_right_mm` (`uint32_t`): Right LiDAR distance in mm (supports $12+\text{ m}$ / $1,200,000\text{ mm}$; `0xFFFFFFFF` = invalid).
    - `dist_left_mm` (`uint32_t`): Left LiDAR distance in mm (supports $12+\text{ m}$ / $1,200,000\text{ mm}$; `0xFFFFFFFF` = invalid).
    - `ball_detected` (`uint8_t`): IR ball presence sensor (`0` = no ball, `1` = ball detected).
  - Trailer (1B): `CRC8` (polynomial `0x07`, init `0x00`) over bytes `2..24`.

## 2. ESP32 Sender Firmware Plan — [TODO]
- **Sensors**:
  - Dual flywheel Hall/optical tachometers (`upper_rpm`, `lower_rpm` as `int16_t`).
  - Geomagnetic / IMU compass (`heading_deg` normalized to $[0, 359]^\circ$ as `int16_t`).
  - Front ($0^\circ$), Right ($+90^\circ$), and Left ($-90^\circ$) LiDAR sensors (`dist_front_mm`, `dist_right_mm`, `dist_left_mm` as `uint32_t` in mm; `0xFFFFFFFF` on timeout/out-of-range).
  - IR ball presence sensor (`ball_detected` as `uint8_t`: `0` = no ball, `1` = ball detected).
- **Transmit Loop**: Non-blocking 50 Hz (`20 ms` interval) serializer writing the 26-byte frame over `Serial1` (`115200 8N1`) to the RPi 5 Host.

---

# Undetermined Questions / Design Decisions (To Address in Next Steps)

## 1. Sensor Placement & Characteristics
- **LiDAR Mounting Offsets**: Exact $(dx, dy)$ relative to robot center once physically measured.
- **Geomagnetic Heading**: Relative vs absolute azimuth calibration before match start.

## 2. Autonomous Routine Storage
- **Chosen Approach**: Interactive **Web UI editor** backed by structured JSON routine definitions stored on the Host (executed by the 1:00 `AUTO` sequence runner).
