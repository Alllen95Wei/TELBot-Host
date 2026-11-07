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

## Planned control framework
Currently, this project has implemented a complete framework for human to send control commands to RPi Pico W "slave" via UART and network; Pico will control the motors based on received UART packets.
For special row, it requires vision processing to detect which one is "active"; but processing would be done by operator's laptop.

# Next steps
We still need additional work on:
- Ability to receive sensor data: our sensors are all attached on an ESP32, which is not planned in the previous progress.
- Pose estimation: Since the robot must be operated in auto for the first minute, it must be able to calculate its current pose on field based on the following data:
  - 3 distance sensors on robot (it's a Lidar, but it only picks 3 directions: front, left, and right)
  - geomagnetic direction
- Mode switching: As mentioned, we haven't designed an auto mode. It shall be able to:
  - executes a fixed process defined via a script or web UI
  - be enabled via web UI
- "Active" field data receiving: Operator's laptop will deploy YOLO and process video stream from RPi, then send back the following data:
```json
{
  "timestamp": 0,
  "active_target": "left/center/right/none",
  "target_detected": false,
}
```

# Undetermined Questions / Design Decisions

## 1. ESP32 Interface & Protocol
- **Physical Connection**: Will ESP32 connect via USB serial (`/dev/ttyUSB0` or `/dev/ttyACM0`) or RPi 5 secondary GPIO UART (`/dev/ttyAMA1`)?
- **Wire Protocol**: 
  - Standard binary framed packet (matching Pico W: `[0xAA, 0x55][LEN][SEQ][CMD=TELEMETRY][PART=SENSORS][PAYLOAD][CRC8]` packed as `<HHHh`)?
  - Or newline-delimited text (`JSON` or `CSV`, e.g. `{"f": 1200, "l": 800, "r": 850, "h": 45.2}\n`)?
- **Streaming Rate & Baud Rate**: What baud rate (e.g. 115200) and publish frequency (e.g. 20 Hz, 50 Hz, 100 Hz)?

## 2. Sensor Placement & Characteristics
- **LiDAR Placement & Offsets**: What are the physical mounting offsets $(dx, dy)$ of the 3 distance sensors relative to the robot's center of rotation?
- **LiDAR Specs & Units**: What is the effective range (min/max in mm or cm) and blind zone?
- **Geomagnetic Heading**: Does the sensor output absolute azimuth ($0^\circ \sim 360^\circ$ relative to magnetic north) or relative angle? Do we need a "Zero Heading / Calibrate" action on the Web UI before starting the match?

## 3. Field Dimensions & Coordinates
- **Field Size**: **Confirmed** — `500 cm` (Top to Bottom, Y-axis) $\times$ `400 cm` (Left to Right, X-axis).
- **Coordinate System**: **Confirmed** — Origin $(0, 0)$ is at the bottom-left corner; the 3x3 target grid is on the top wall ($y = 500\text{ cm}$, $0^\circ$ heading faces Top).
- **Target Grid Location**: Exact horizontal X-coordinates / dimensions of the 3x3 goal grid along the top wall ($y = 500\text{ cm}$) to be measured.
- **Obstacle Rejection**: **Confirmed** — `PoseEstimator` automatically records the latest valid distances of `L` and `R`; when dual-beam span validation fails, it computes the delta between the previous valid value and the current reading for each side, using only the beam with the smaller delta.

## 4. Autonomous Mode & Workflow
- **Auto Routine Storage**: Should the routine be defined in a JSON file (e.g. `auto_routines.json`), a Python script, or editable directly in the Web UI?
- **Motion Control**: Should Auto use closed-loop target pose navigation ($x, y, \theta$ P/PID controller), timed open-loop drive, or a hybrid sequence?
- **Match Transition**: **Confirmed** — When 1:00 Auto expires, robot immediately stops all motors and enters `AUTO_WAIT`. Robot does not start Tele-Op automatically; operator must manually trigger "Enable Tele-Op" from Web UI to begin the 2:00 Tele-Op period.

