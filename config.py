class UART:
    PORT = "/dev/ttyAMA0"
    BAUD_RATE = 115200
    TIMEOUT = 0.1  # seconds (responsive for RX thread polling)


class Server:
    HOST = "0.0.0.0"
    PORT = 8000


class Safety:
    PING_INTERVAL = 0.2  # seconds (200 ms heartbeat; Pico watchdog is 400 ms)
    DRIVE_TIMEOUT = 0.5  # seconds (auto-zero chassis drive if commands stop arriving)


class EStop:
    CHIP = "/dev/gpiochip0"  # RPi 5 RP1 40-pin header (pinctrl-rp1)
    PIN = 17  # BCM GPIO pin wired between E-Stop button and GND
    NORMALLY_CLOSED = True  # Closed to GND (LOW) = normal; Open (HIGH) = E-Stop triggered
    DEBOUNCE_MS = 20  # Hardware/kernel debounce interval in milliseconds
