# BBAI64 ROS Arduino Bridge Firmware

> **Production Differential-Drive Motor Controller Firmware with Real-Time Traction Control (TCS) & e-LSD**  
> Tailored for Arduino Nano (ATmega328P / CH340), Cytron MDD10A Dual 10A Motor Driver, and BeagleBone AI-64 ROS 2 Architecture.

---

## 1. System Overview

This repository contains the hardened Arduino firmware that bridges high-level ROS 2 velocity commands (`/cmd_vel`) from a single-board computer (BeagleBone AI-64) into deterministic low-level motor PWM and directional signals, while polling dual optical/magnetic wheel encoders to stream accurate odometry.

```
+--------------------------------------------------------------------------+
|                          BeagleBone AI-64 (ROS 2)                        |
|                                                                          |
|       /cmd_vel (Twist)                        /odom & /encoders          |
|              |                                       ^                   |
|              v                                       |                   |
|     ASCII Serial Protocol: 'm <L> <R>' or 'o <L> <R>' | 'e' -> Enc Ticks  |
+--------------------------------------------------------------------------+
                                    |  (57600 baud over /dev/ttyUSB0)
                                    v
+--------------------------------------------------------------------------+
|                       Arduino Nano (ATmega328P)                          |
|                                                                          |
|   - 30 Hz Closed-Loop Velocity Integrator                                |
|   - Real-Time Traction Control System (TCS) & Wheel Spin Detection       |
|   - Electronic Limited-Slip Differential (e-LSD) Straight Lock           |
|   - Slew Rate Limiter (PWM_RAMP_STEP = 15) & Soft Breakaway (MIN_PWM 35)  |
|   - 1.0s Auto-Stop Watchdog Protection                                   |
+--------------------------------------------------------------------------+
          |                                                    ^
          | (PWM & DIR Lines)                                  | (PCINT1/PCINT2)
          v                                                    |
+-----------------------------------+             +------------------------+
|  Cytron MDD10A Dual Motor Driver  |             |  Dual Wheel Encoders   |
|  - Left Channel:  D5 (PWM), D6(DIR)|             |  - Left:  D2/D3 (PORTD)|
|  - Right Channel: D10(PWM), D9(DIR)|             |  - Right: A4/A5 (PORTC)|
+-----------------------------------+             +------------------------+
```

---

## 2. Hardware Pin Assignments & Truth Table

### A. Arduino Nano Pin Connections
| Function | Arduino Pin | Port / Timer Register | Cytron MDD10A Pin |
| :--- | :---: | :---: | :---: |
| **Left Motor PWM** | **D5** | PORTD5 (Timer0 OCR0B) | `PWM 1` |
| **Left Motor DIR** | **D6** | PORTD6 (Digital Out) | `DIR 1` |
| **Right Motor PWM** | **D10** | PORTB2 (Timer1 OCR1B) | `PWM 2` |
| **Right Motor DIR** | **D9** | PORTB1 (Digital Out) | `DIR 2` |
| **Left Encoder A** | **D2** | PORTD2 (PCINT18) | Encoder L Phase A |
| **Left Encoder B** | **D3** | PORTD3 (PCINT19) | Encoder L Phase B |
| **Right Encoder A**| **A4** | PORTC4 (PCINT12) | Encoder R Phase A |
| **Right Encoder B**| **A5** | PORTC5 (PCINT13) | Encoder R Phase B |
| **Heartbeat LED** | **D13**| PORTB5 | Onboard LED (1 Hz toggle) |

---

### B. Cytron MDD10A Control Truth Table
Unlike obsolete L298N dual H-bridges that require 4 direction pins and 2 enable pins, the Cytron MDD10A uses a dedicated `PWM` and `DIR` pin per motor channel.

| Motor | Commanded Motion | Logic State | Encoder Expected Behavior |
| :--- | :--- | :--- | :--- |
| **Left Motor** | **Forward** | `DIR (D6) = HIGH`, `PWM (D5) = speed` | Left ticks increase positive ($\Delta L > 0$) |
| **Left Motor** | **Reverse** | `DIR (D6) = LOW`, `PWM (D5) = speed` | Left ticks decrease negative ($\Delta L < 0$) |
| **Left Motor** | **Stop / Park** | `DIR (D6) = LOW`, `PWM (D5) = 0` | Zero torque, fast braking |
| **Right Motor** | **Forward** | `DIR (D9) = LOW`, `PWM (D10) = speed` | Right ticks increase positive ($\Delta R > 0$) |
| **Right Motor** | **Reverse** | `DIR (D9) = HIGH`, `PWM (D10) = speed` | Right ticks decrease negative ($\Delta R < 0$) |
| **Right Motor** | **Stop / Park** | `DIR (D9) = LOW`, `PWM (D10) = 0` | Zero torque, fast braking |

---

## 3. Advanced Features & Upgrades Over Stock Bridge

1. **Cytron MDD10A Mode**: Dedicated `#elif defined CYTRON_MOTOR_DRIVER` implementation ensuring all PWM and DIR pins start in a safe `LOW` state at boot.
2. **Servo Conflict Resolution**: Stock `ROSArduinoBridge` enabled `#define USE_SERVOS`, which hijacked Pin 3 for servo output, clashing with Left Encoder interrupt on D3 and disabling Timer1 PWM on D10. Servos are disabled (`#undef USE_SERVOS`), restoring encoder and motor PWM stability.
3. **Soft Breakaway & Slew Limiting**:
   - `MIN_BASE_PWM = 35` (~14% duty cycle) overcomes static friction gently without harsh torque shock.
   - `PWM_RAMP_STEP = 15` per 30 Hz loop provides progressive acceleration and deceleration.
   - `MAX_PWM = 220` (~86% duty cycle) safeguards motors and drivers against overcurrent.
4. **Traction Control System (TCS) & e-LSD**:
   - **Wheel-Spin Detection**: Compares instantaneous wheel speed against target speed (`TCS_SLIP_TOLERANCE 4`).
   - **Acceleration Slew Guard**: Detects sudden unconstrained wheel spin (`TCS_MAX_ACCEL 8`).
   - **Anti-Runaway Direction Guard**: If a wheel spins opposite to commanded velocity vector, output is clamped to base PWM, preventing positive feedback spinouts.
   - **Electronic Limited-Slip Differential (e-LSD)**: Dynamically equalizes speed divergence during straight travel (`TCS_DIFF_SLIP_THRESHOLD 5`).
5. **Real-Time Diagnostic Query (`q`)**:
   - Querying `q` returns timer OCR registers (`OCR0B`, `OCR1B`), pin digital reads, current vs target PWM, raw encoder counts, and active TCS flags.

---

## 4. Serial Command Protocol Reference

All commands must be terminated by a **Carriage Return (`\r`, ASCII 13)**.

| Command | Arguments | Example | Description | Response |
| :---: | :---: | :---: | :--- | :--- |
| `b` | none | `b\r` | Read serial baud rate | `57600\r\n` |
| `e` | none | `e\r` | Read encoder counts | `<left_ticks> <right_ticks>\r\n` |
| `r` | none | `r\r` | Reset encoder counts to zero | `OK\r\n` |
| `o` | `<left> <right>` | `o 120 120\r` | Direct raw PWM speed (-220 to 220) | `OK\r\n` |
| `m` | `<left> <right>` | `m 10 10\r` | Closed-loop target ticks per 30Hz frame | `OK\r\n` |
| `u` | `<p>:<d>:<i>:<o>` | `u 20:12:0:50\r`| Update PID gains | `OK\r\n` |
| `q` | none | `q\r` | Query detailed hardware timer & TCS diagnostics | Diagnostic string |

---

## 5. Quick Verification via Python

```python
import serial, time

ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1.0)
time.sleep(2.0)  # Wait for Arduino reset

# Check baudrate
ser.write(b'b\r')
print("Baud:", ser.readline().decode().strip())

# Read encoders
ser.write(b'e\r')
print("Encoders:", ser.readline().decode().strip())

# Forward pulse at 90 PWM for 1 second
ser.write(b'o 90 90\r')
time.sleep(1.0)
ser.write(b'o 0 0\r')

# Check diagnostics
ser.write(b'q\r')
print("Diag:", ser.readline().decode().strip())

ser.close()
```

---

## 6. License

Released under the **BSD-3-Clause License**.
