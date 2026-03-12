# Pin Configuration

## Overview

This document describes the hardware pin connections used in the Micromouse robot.

The system is built around an **Arduino Nano**, interfacing with:

* TB6612FNG motor driver
* Quadrature wheel encoders
* Five VL53L0X time-of-flight sensors

Proper pin mapping is critical for correct motor control, encoder feedback, and sensor communication.

---

# Microcontroller

Board: **Arduino Nano (ATmega328P)**

Communication Interfaces:

* I2C → VL53L0X sensors
* Digital IO → motor driver
* Interrupt pins → encoders
* PWM outputs → motor speed control

---

# Motor Driver (TB6612FNG)

The robot uses a **dual H-bridge motor driver** to control the left and right motors.

| Pin  | Arduino Pin | Function                      |
| ---- | ----------- | ----------------------------- |
| STBY | D9          | Motor driver standby control  |
| AIN1 | D11         | Left motor direction control  |
| AIN2 | D4          | Left motor direction control  |
| PWMA | D5          | Left motor PWM speed control  |
| BIN1 | D6          | Right motor direction control |
| BIN2 | D7          | Right motor direction control |
| PWMB | D8          | Right motor PWM speed control |

### Notes

* `STBY` must be set HIGH to enable the motor driver.
* PWM signals control motor speed.

---

# Wheel Encoders

Quadrature encoders provide feedback for measuring wheel rotation.

| Encoder Signal  | Arduino Pin | Description         |
| --------------- | ----------- | ------------------- |
| Left Encoder A  | D3          | Interrupt input     |
| Left Encoder B  | A3          | Direction detection |
| Right Encoder A | D2          | Interrupt input     |
| Right Encoder B | A2          | Direction detection |

### Notes

Pins **D2 and D3** are used because they support **hardware interrupts**, which allows accurate encoder pulse detection.

---

# Distance Sensors (VL53L0X)

Five VL53L0X sensors are used to detect maze walls.

| Sensor      | XSHUT Pin | Purpose                       |
| ----------- | --------- | ----------------------------- |
| Front       | D12       | Detect obstacle ahead         |
| Front Left  | A1        | Detect approaching left wall  |
| Front Right | A0        | Detect approaching right wall |
| Left        | D10       | Wall-following control        |
| Right       | D13       | Wall-following control        |

### XSHUT Usage

All VL53L0X sensors share the same default I2C address.

To use multiple sensors on the same bus:

1. All sensors are first disabled using their **XSHUT pins**
2. Sensors are enabled **one at a time**
3. Each sensor is assigned a **new I2C address**

This prevents address conflicts on the I2C bus.

---

# I2C Bus

The VL53L0X sensors communicate via the I2C interface.

| Signal | Arduino Pin |
| ------ | ----------- |
| SDA    | A4          |
| SCL    | A5          |

The I2C clock speed is configured to **400 kHz** for faster sensor communication.

---

