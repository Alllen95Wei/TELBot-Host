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
