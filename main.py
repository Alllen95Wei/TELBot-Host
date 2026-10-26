import serial
from time import sleep

from config import *

ser = serial.Serial(
    UART.PORT, 
    UART.BAUD_RATE, 
    timeout=UART.TIMEOUT,
)

def read_serial():
    print(f"(R waiting: {ser.in_waiting})")
    if ser.in_waiting > 0:
        data = ser.readline().strip().decode()
        print(f"R: {data}")

def write_serial():
    ser.write(b"Hello from Raspberry Pi!\n")
    print("W: Hello from Raspberry Pi!")

def main():
    while True:
        write_serial()
        read_serial()
        sleep(0.5)

if __name__ == "__main__":
    main()
