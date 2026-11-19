class UART:
    PORT = "/dev/ttyAMA0"
    BAUD_RATE = 115200
    TIMEOUT = 0.1  # seconds (responsive for RX thread polling)


class ESP32:
    PORT = "/dev/ttyAMA2"
    BAUD_RATE = 115200
    TIMEOUT = 0.05  # seconds (responsive for 50 Hz RX thread polling)
    STALE_TIMEOUT_S = 0.5  # seconds before ESP32 sensor snapshot is marked stale


class Server:
    HOST = "0.0.0.0"
    PORT = 8000
    SSL_CERTFILE = "certs/server.crt"
    SSL_KEYFILE = "certs/server.key"
    CA_CERTFILE = "certs/ca.crt"


class Safety:
    PING_INTERVAL = 0.2  # seconds (200 ms heartbeat; Pico watchdog is 400 ms)
    DRIVE_TIMEOUT = 1    # seconds (auto-zero chassis drive if commands stop arriving)


class EStop:
    CHIP = "/dev/gpiochip0"  # RPi 5 RP1 40-pin header (pinctrl-rp1)
    PIN = 17  # BCM GPIO pin wired between E-Stop button and GND
    NORMALLY_CLOSED = True  # Closed to GND (LOW) = normal; Open (HIGH) = E-Stop triggered
    DEBOUNCE_MS = 20  # Hardware/kernel debounce interval in milliseconds


class Camera:
    DEVICE = "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920-video-index0"
    FALLBACK_DEVICE = "/dev/video0"
    HOST = "0.0.0.0"
    PORT = 8001
    DEFAULT_PRESET = "720p15"
    BUFFERS = 2  # minimal buffer latency (2 frames)
    TCP_NODELAY = True
    ALLOW_ORIGIN = "*"


class Field:
    # Origin (0, 0) is at the bottom-left corner; target 3x3 grid is at the top wall (y = LENGTH_CM)
    WIDTH_CM = 400.0   # X-axis: Left (0.0) to Right (400.0) in cm
    LENGTH_CM = 500.0  # Y-axis: Bottom (0.0) to Top (500.0) in cm


class PoseSensor:
    # LiDAR mounting offsets (dx, dy) in cm relative to robot center (0, 0)
    # Robot body frame: +x is right, +y is front (update once physically measured)
    FRONT_OFFSET_X_CM = 0.0
    FRONT_OFFSET_Y_CM = 0.0
    LEFT_OFFSET_X_CM = 0.0
    LEFT_OFFSET_Y_CM = 0.0
    RIGHT_OFFSET_X_CM = 0.0
    RIGHT_OFFSET_Y_CM = 0.0

    # Valid LiDAR range limits (in cm)
    MIN_RANGE_CM = 2.0
    MAX_RANGE_CM = 600.0

    # Heading convention: 0 deg = facing Top wall (+Y, toward 3x3 grid); True = clockwise azimuth
    CLOCKWISE_HEADING = True
    DEFAULT_HEADING_OFFSET_DEG = 0.0

    # Obstacle rejection & wall-projection thresholds
    SPAN_TOLERANCE_CM = 30.0       # Max |x_L - x_R| discrepancy before L/R validation fails
    MAX_RAY_INCIDENCE_DEG = 35.0   # Max angle off wall normal to project a ray onto that wall
    MAX_FRONT_DELTA_CM = 80.0      # Max sudden single-step jump on Front ray before rejecting as obstacle
    MAX_RIGHT_DELTA_CM = 80.0      # Max sudden single-step jump on Right ray before rejecting as obstacle


class Match:
    # Game phase durations in seconds
    AUTO_DURATION_S = 60.0      # 1:00 Autonomous
    TELEOP_DURATION_S = 120.0   # 2:00 Tele-Op

    # Operator safety & match transition flags
    LOCK_CONTROLLER_IN_AUTO = True
    REQUIRE_MANUAL_TELEOP_START = True  # After auto, operator must explicitly enable Tele-Op
    AUTO_SWITCH_MODE = False


class SpecialGrid:
    """Special 1x3 goal row along the top wall (Y = 500 cm) active during Autonomous."""
    COLUMN_SPACING_CM = 60.0    # Gap between columns
    CENTER_X_CM = 200.0         # Center column X coordinate
    TOP_Y_CM = 500.0            # Top wall Y coordinate
    GRID_WIDTH_CM = 45.0        # Physical width of each goal
    GRID_HEIGHT_CM = 35.0       # Physical height of each goal

    TARGET_COORDS = {
        "left": (CENTER_X_CM - COLUMN_SPACING_CM, TOP_Y_CM),    # (140.0, 500.0)
        "center": (CENTER_X_CM, TOP_Y_CM),                     # (200.0, 500.0)
        "right": (CENTER_X_CM + COLUMN_SPACING_CM, TOP_Y_CM),   # (260.0, 500.0)
    }

    @classmethod
    def get_coords(cls, target: str) -> tuple[float, float] | None:
        return cls.TARGET_COORDS.get(str(target).lower().strip())


class ShooterPID:
    """Closed-loop dual flywheel RPM PID + feedforward configuration."""

    MAX_RPM = 5500.0
    CONTROL_LOOP_HZ = 50.0
    OUT_MIN = 0.0
    OUT_MAX = 1000.0

    # Independent upper flywheel PID + feedforward gains
    UPPER_KP = 0.15
    UPPER_KI = 0.08
    UPPER_KD = 0.002
    UPPER_KV = 1000.0 / 5500.0
    UPPER_KS = 0.0

    # Independent lower flywheel PID + feedforward gains
    LOWER_KP = 0.15
    LOWER_KI = 0.08
    LOWER_KD = 0.002
    LOWER_KV = 1000.0 / 5500.0
    LOWER_KS = 0.0

    # Anti-windup, derivative filter, and readiness tolerances
    I_ZONE_RPM = 400.0
    MAX_INTEGRAL = 1500.0
    D_FILTER_ALPHA = 0.3
    RPM_TOLERANCE = 100.0
    DEFAULT_MANUAL_RPM = 4400.0  # Matches default UI slider (800 / 1000 * 5500)

    # Distance (cm) -> Target RPM piecewise-linear lookup table
    DISTANCE_TO_RPM_TABLE: tuple[tuple[float, float], ...] = (
        (80.0, 2400.0),
        (140.0, 2950.0),
        (200.0, 3450.0),
        (260.0, 3900.0),
        (320.0, 4350.0),
        (380.0, 4750.0),
        (450.0, 5200.0),
    )


class ConveyorAutoReverse:
    """Automatic spinner anti-jam reverse when shooter is running with no ball detected."""

    ENABLED = True
    NO_BALL_DELAY_S = 0.5      # Continuous no-ball time while shooting before auto-reverse triggers
    REVERSE_DURATION_S = 0.4   # Duration of each automatic reverse pulse
    COOLDOWN_S = 1.0           # Normal/forward dwell time before triggering another pulse if still no ball
