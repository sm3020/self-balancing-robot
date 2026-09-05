# Self-Balancing Robot
Arduino-based self-balancing robot using MPU6050 tilt sensing and closed-loop PID control to dynamically correct wheel motion and maintain balance.

## Overview
The robot is a 2-wheeled self-balancing system which behaves like an inverted pendulum, meaning it is naturally unstable so needs continuous active correction to stay upright. I used an MPU6050 inertial measurement unit to measure the robot's orientation using accelerometer & gyroscope data. The robot's tilt is continuously compared with the desired upright position. A closed-loop PID controller calculates the necessary corrective response and I used an L298N motor driver to respond by controlling the 2 DC motors which move the wheels in the same direction as the gall to bring the wheelbase back underneath the robot's centre of mass. This project gave me practical experience with IMU sensing, feedback control, PID tuning, motor control and C++ programming. 

## Control Principle
The robot is modelled to be an inverted pendulum, which is inherently unstable. The MPU6050 measures the robot's angular position and velocity which allows the PID controller to determine its deviation from the upright equilibrium position. 

The PID controller uses this error to determine the required motor response:

e(t) = θ_target - θ_measured

The motors then drive the wheels in the direction of the fall, returning the bottom of the robot's body beneath the robot's centre of mass and restoring balance. 

## Hardware

The robot was built using the following components:
- ELEGOO UNO R3 Board (an Arduino-compatible microcontroller)
- MPU6050 6-axis inertial measurement unit (accelerometer + gyroscope)
- L298N motor driver
- 2 x DC geared motors
- 2 X wheels
- 9V battery
- Two-wheeled 3D printed chassis
- Jumper wires

The MPU6050 provides the robot's motion and orientation measurements, while the microcontroller processes this data and runs the PID control algorithm. The L298N motor driver allows the microcontroller to control the direction and speed of the 2 DC motors. 

## Feedback loop
MPU6050 -> Tilt measurement -> PID Controller -> Motor command -> L298N Motor Driver -> DC motors -> Robot movement, then a new tilt measurement is taken

<img width="1247" height="1003" alt="image" src="https://github.com/user-attachments/assets/c8d441e1-6d75-4ef7-97e5-d42ad4d1de5c" />

## Software
This loop runs repeatedly to maintain balance:
1. Reads accelerometer and gyroscope measurements from the MPU6050 IMU
2. Estimates the robot's tilt angle
3. Calculates the error between the measured angle and upright equilibrium position
4. Passes this error into the PID controller
5. Converts the PID output into motor speed and direction commands
6. Sends these commands to the L298N motor driver

## PID Tuning 
The PID controller was tuned to react fast enough without excessive oscillation. 
- **Proportional term (P):** provides a correction proportional to the current tilt error
- **Integral term (P):** accumulates error over time, helping to eliminate small persistent errors that the proportional term alone won't correct
- **Derivative term (P):** responds to how quickly the error is changing, helps avoid overshoot and oscillation

## Challenges & Debugging 

Some of my main challenges included:
- Ensuring the motors rotated in the correct direction relative to the measured tilt
- Ensuring both motors produced sufficiently similar responses
- Tuning the PID settings so that the robot reacts fast enough without becoming unstable

## Future improvements 
- Design a custom PCB to reduce wiring and potential faults
- Add wireless control/telemetry 












