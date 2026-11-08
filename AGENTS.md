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

# Undone Tasks

1. **PID Controllers**:
   - Closed-loop Chassis Pose-to-Point PID for autonomous navigation.
   - Closed-loop Dual Flywheel RPM PID with automatic distance-to-RPM calculation and manual fallback.
2. **ESP32 Sensor Interface & Telemetry**:
   - Read 3 directional LiDARs, geomagnetic heading, and upper/lower flywheel RPM sensors from ESP32 into RPi 5 Host (to be designed and implemented after PID control).
3. **Autonomous Routine Storage & Workflow**:
   - Sequence runner executing fixed steps during the 1:00 Auto phase (e.g. drive to point $\to$ align $\to$ spin up flywheels $\to$ feed balls).

---

# PID Controller Implementation Plan

## 1. Reusable PID Core (`core/pid.py`)
- Generic `PIDController` class supporting position and velocity control with feedforward:
  $$u(t) = K_v \cdot r(t) + K_s \cdot \operatorname{sgn}(r(t)) + K_p \cdot e(t) + K_i \int e(\tau)\,d\tau - K_d \frac{dy(t)}{dt}$$
- **Safeguards**:
  - Derivative-on-measurement to prevent setpoint change kicks.
  - Output clamping `[out_min, out_max]` (specifically `out_min = 0` for flywheels to prevent any negative output).
  - Anti-windup via integral clamping, `i_zone`, and integrator freeze on saturation.
  - Continuous angle wrapping for heading control across $0^\circ / 360^\circ$ boundary.
  - Immediate `reset()` when target is 0 or subsystem is stopped.

## 2. Chassis Pose-to-Point PID (`ChassisPoseController` in `core/control.py`)
- **Inputs**: Current field pose $(x_{\text{cm}}, y_{\text{cm}}, \theta_{\text{deg}})$ from `PoseEstimator.latest_pose`.
- **Target**: `(target_x_cm, target_y_cm, target_heading_deg)`.
- **3 Independent Axes**:
  - $X$-axis PID: $e_x = x_{\text{target}} - x_{\text{cm}} \to u_X$
  - $Y$-axis PID: $e_y = y_{\text{target}} - y_{\text{cm}} \to u_Y$
  - Heading PID: wrapped angular error $e_\theta \to \omega$
- **Coordinate Transformation**: Rotate field velocities $(u_X, u_Y)$ into robot body frame ($v_x$ forward, $v_y$ strafe) matching $0^\circ = +Y$, $90^\circ = +X$:
  $$v_x = u_Y \cos\theta + u_X \sin\theta, \qquad v_y = u_X \cos\theta - u_Y \sin\theta$$
- **Safety**: Auto-zero output on stale/invalid pose; `is_at_target()` check with configurable tolerances.

## 3. Dual Flywheel RPM PID + Distance Auto-Speed (`ShooterSpeedController` in `core/control.py`)
- **Shared Target**: Operator and auto-distance logic produce **one shared setpoint** ($0 \sim 5500$ RPM).
- **Dual Independent Regulation**: Two independent PID instances (`upper_pid`, `lower_pid`) regulate upper and lower flywheels separately against the shared target to compensate for physical motor/ESC differences:
  - Outputs clamped to $[0, 1000]$ (no negative output / reverse braking).
  - Target 0 immediately returns $(0, 0)$ and resets integrators.
- **Distance-Based Auto-Speed**:
  - Computes Euclidean distance $d$ from robot pose to target grid along the top wall ($y = 500\text{ cm}$).
  - Evaluates configurable piecewise-linear interpolation table in `config.py` to set target RPM.
  - Operates in **Auto (Distance)** mode by default; operator adjusting manual slider or Cross Pad D-pad switches to **Manual Override**.
- **Pre-ESP32 Sensor Fallback**:
  - When ESP32 live RPM telemetry is present: full closed-loop PID.
  - Before ESP32 is integrated or when RPM data is stale: open-loop feedforward fallback ($\text{cmd} = \text{round}(1000 \times \text{target\_rpm} / 5500)$) to prevent integral windup against missing feedback.

## 4. Protocol & Slave Firmware Updates
- **UART Framing (`PART_SHOOTER`)**:
  - Full shooter packet: 6 bytes (`<hhBB`) $\to$ `(upper_flywheel, lower_flywheel, angle_run, angle_reversed)`.
  - Flywheel-only packet: 4 bytes (`<hh`) $\to$ `(upper_flywheel, lower_flywheel)`.
  - Angle-only (3 bytes, `<BBB`) and Disable-angle (1 byte, `<B`) remain distinct and non-colliding.
- **Pico W `Shooter` Subsystem**: Update `start_shoot(upper_speed: float, lower_speed: float | None = None)` to drive `upper_flywheel` and `lower_flywheel` independently.

## 5. Host Integration & Web UI
- **Control Loop in `main.py`**: 50 Hz control tick coordinating active PIDs, match phase gating, and UART transmissions.
- **Web Console**:
  - Single shared speed control (slider + Cross Pad D-pad `4`/`20` step adjustment).
  - Mode toggle: Auto (Distance) vs. Manual.
  - Telemetry readouts for target distance, target RPM, and live upper/lower RPMs.
  - `#battery-text` indicator displaying current shooter speed percentage.

---

# Undetermined Questions / Design Decisions (To Address in Next Steps)

## 1. ESP32 Interface & Protocol (Next Step after PID)
- **Physical Connection**: USB serial (`/dev/ttyUSB0` or `/dev/ttyACM0`) vs RPi 5 secondary GPIO UART (`/dev/ttyAMA1`).
- **Wire Protocol**: Binary framed packet matching Pico W vs newline-delimited JSON.
- **Streaming Rate & Baud Rate**: 115200 baud, 20 Hz / 50 Hz.

## 2. Sensor Placement & Characteristics
- **LiDAR Mounting Offsets**: Exact $(dx, dy)$ relative to robot center once physically measured.
- **LiDAR Specs**: Range and blind zones.
- **Geomagnetic Heading**: Relative vs absolute azimuth calibration before match start.

## 3. Autonomous Routine Storage
- Routine definition format: JSON sequence file vs Python script vs interactive Web UI editor.
