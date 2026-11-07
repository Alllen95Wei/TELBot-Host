class UART:
    PORT = "/dev/ttyAMA0"
    BAUD_RATE = 115200
    TIMEOUT = 0.1  # seconds (responsive for RX thread polling)


class Server:
    HOST = "0.0.0.0"
    PORT = 8000
    SSL_CERTFILE = "certs/server.crt"
    SSL_KEYFILE = "certs/server.key"
    CA_CERTFILE = "certs/ca.crt"


class Safety:
    PING_INTERVAL = 0.2  # seconds (200 ms heartbeat; Pico watchdog is 400 ms)
    DRIVE_TIMEOUT = 0.5  # seconds (auto-zero chassis drive if commands stop arriving)


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


class Match:
    # Game phase durations in seconds
    AUTO_DURATION_S = 60.0      # 1:00 Autonomous
    TELEOP_DURATION_S = 120.0   # 2:00 Tele-Op

    # Operator safety & match transition flags
    LOCK_CONTROLLER_IN_AUTO = True
    REQUIRE_MANUAL_TELEOP_START = True  # After auto, operator must explicitly enable Tele-Op
