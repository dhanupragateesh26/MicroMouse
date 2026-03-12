# Micromouse Navigation — Version 1 (Wall Following)

## Overview

This version implements a **basic autonomous navigation algorithm for a Micromouse robot using wall-following control**.

The robot navigates a **10×10 cm grid maze** using five VL53L0X distance sensors to detect nearby walls and maintain alignment inside maze corridors.

Movement is executed using **encoder-based motion control**, while steering corrections are performed using a **proportional-derivative wall-following controller**.

---

## Hardware Configuration

| Component        | Description                        |
| ---------------- | ---------------------------------- |
| Microcontroller  | Arduino Nano                       |
| Motor Driver     | TB6612FNG                          |
| Distance Sensors | 5 × VL53L0X Time-of-Flight sensors |
| Motors           | Dual DC gear motors                |
| Encoders         | Quadrature wheel encoders          |

---

## Sensor Layout

The robot uses five sensors to detect surrounding walls:

Front – detects obstacles ahead
Front Left – anticipates approaching walls
Front Right – anticipates approaching walls
Left – used for wall-following control
Right – used for wall-following control

This configuration allows the robot to maintain accurate alignment within maze corridors.

---

## Navigation Strategy

The robot uses a **reactive wall-following algorithm**.

At each cell the robot evaluates surrounding walls and selects an action.

### Possible Actions

* Move Forward
* Turn Left
* Turn Right
* Turn Back

### Decision Priority

Forward → Left → Right → Reverse

This allows the robot to explore unknown maze environments.

---

## Motion Control

Movement is executed in **fixed 100 mm increments**, corresponding to the size of one maze cell.

Distance traveled is estimated using wheel encoders.

Conversion from distance to encoder ticks:

* wheel circumference
* gear ratio
* encoder pulses per revolution

These parameters allow accurate linear motion estimation.

---

## Wall Following Control

To remain centered within maze corridors, the robot uses a **PD wall-following controller**.

Error is calculated from the difference between left and right wall distances.

control = Kp × error + Kd × change_in_error

The controller adjusts the motor speeds to steer the robot toward the center of the corridor.

---

## Motor Control

Motor control is implemented using the **TB6612FNG dual H-bridge driver**.

Speed control is performed using PWM signals, while direction is determined through digital control pins.

This configuration allows:

* forward motion
* reverse motion
* differential steering
* in-place rotation

---


## Purpose

This implementation serves as the **baseline navigation system** for the micromouse robot, providing reliable maze traversal using wall-following behavior.
