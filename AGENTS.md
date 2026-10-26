# About this project
This project is aiming to build communications between robot operators, robot sensors, and co-processor (RPi Pico W) on the robot.

## Structure in real life
Our robot has the following design:
- RPi 5 (This machine) plays as "host". It should be able to:
  - receive the commands from human and send standardized data to slave;
  - receive sensor data from slave, process them, and adjust/calculate the signal sent to the motors (via slave).
- RPi Pico W plays as "slave". It should be able to:
  - read commands from host and convert them into signals to motor;
  - constantly read data from sensors and send them back to host.
> The communication between RPi 5 and RPi Pico W is using UART.
> Since Pico needs a different language, we don't implement its code in this project. We only need to focus on the "host".

# Technical targets & aimings
1. TX and RX in UART should be able to process in real-time. That is, making TX an RX two parallel jobs that keep running.
2. User should be able to control the robot via a web panel.
  - The web should use `WebHID` API to read joysticks' inputs, and send the signal back using API.
  - As mentioned, an lightweight API server should be hosted.