# BeagleBone AI-64 Arduino Firmware Compilation, Flashing & Calibration Guide

## 1. Hardware Architecture & Wiring

### A. Power & Ground Isolation
- **Motor Power (12V / 24V)**: Connect battery or power source directly to Cytron MDD10A `POWER +` and `POWER -` screw terminals.
- **Microcontroller Power (5V)**: The Arduino Nano is powered via the USB connection to the BeagleBone AI-64.
- **Common Ground**: Connect the `GND` pin of the Cytron MDD10A to the Arduino Nano `GND` pin to maintain a shared logic reference.

### B. Pinout Schematic Summary
```
Arduino Nano (ATmega328P)        Cytron MDD10A
-------------------------        -------------
Pin D5 (Timer0 PWM)       -----> PWM 1 (Left Speed)
Pin D6 (Digital Out)      -----> DIR 1 (Left Direction)
Pin D10 (Timer1 PWM)      -----> PWM 2 (Right Speed)
Pin D9 (Digital Out)      -----> DIR 2 (Right Direction)
GND                       -----> GND

Arduino Nano (ATmega328P)        Left & Right Wheel Encoders
-------------------------        ---------------------------
Pin D2 (PORTD PCINT18)    <----- Left Encoder Phase A
Pin D3 (PORTD PCINT19)    <----- Left Encoder Phase B
Pin A4 (PORTC PCINT12)    <----- Right Encoder Phase A
Pin A5 (PORTC PCINT13)    <----- Right Encoder Phase B
5V                        -----> Encoder VCC (5V)
GND                       -----> Encoder GND
```

> [!IMPORTANT]
> **Pin Change Interrupts (PCINT)**: Pins D2/D3 are on PORTD, monitored by `PCMSK2` and `PCIE2`. Pins A4/A5 are on PORTC, monitored by `PCMSK1` and `PCIE1`. The firmware enables internal pull-ups on these pins (`PORTD |= ...`, `PORTC |= ...`), eliminating the need for external pull-up resistors.

---

## 2. Compiling & Flashing

### Option 1: Directly on the BeagleBone AI-64 using `arduino-cli` (Recommended)

1. **Install arduino-cli and AVR core** (if not already installed):
```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/
arduino-cli core update-index
arduino-cli core install arduino:avr
```

2. **Compile the Sketch**:
```bash
cd bbai64_arduino_firmware/ROSArduinoBridge
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old .
# If you have a newer bootloader Nano, use:
# arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 .
```

3. **Flash to Arduino via USB**:
```bash
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old .
```

---

### Option 2: Flashing from Windows via Arduino IDE

1. Open `bbai64_arduino_firmware/ROSArduinoBridge/ROSArduinoBridge.ino` in Arduino IDE.
2. Under **Tools**:
   - **Board**: `Arduino Nano`
   - **Processor**: `ATmega328P (Old Bootloader)` *(or `ATmega328P` for newer chips)*
   - **Port**: Select the corresponding COM port (e.g. `COM3` or `COM4`).
3. Click **Upload** (<kbd>Ctrl+U</kbd>).

---

## 3. Deep Dive: Resolved Issues & Architecture Fixes

### Issue 1: Servo Library Hijacking Timer1 & Pin 3
- **Root Cause**: The stock `ROSArduinoBridge` had `#define USE_SERVOS` enabled. The AVR `Servo.h` library attaches hardware Timer1 to Pin 3 (`PD3`), disabling Timer1 PWM on Pin 10 (`OCR1B` for Right Motor) and conflicting with the Left Encoder pin change interrupt on `D3`.
- **Solution**: `#undef USE_SERVOS` in `ROSArduinoBridge.ino`. Servos are disabled, giving full exclusive use of Timer1 to Motor PWM and restoring interrupt fidelity on D3.

### Issue 2: Violent 360° Spin in Reverse
- **Root Cause**: An asymmetrical driving logic was previously asserting `D10 HIGH, D9 PWM` on reverse for the Right motor. On Cytron MDD10A, this forced a motor stall (-24 ticks) while the left wheel reversed (-4670 ticks), causing an immediate clockwise spinout.
- **Solution**: Cytron MDD10A truth table alignment:
  - Left Motor: `DIR (D6) HIGH` = Forward, `DIR (D6) LOW` = Reverse, `PWM on D5`.
  - Right Motor: `DIR (D9) LOW` = Forward, `DIR (D9) HIGH` = Reverse, `PWM on D10`.
  Both wheels reverse cleanly with synchronized negative encoder delta ticks.

### Issue 3: Sudden Acceleration Burst on Direction Changes
- **Root Cause**: `MIN_BASE_PWM` was previously set to `80` (31% duty cycle), causing a harsh torque snap whenever teleop commenced.
- **Solution**: Reduced `MIN_BASE_PWM` to `35` (~14% duty cycle) and tuned `PWM_RAMP_STEP` to `15` per 33ms frame for gentle, progressive breakaway.

---

## 4. Calibration & Tuning Guide

### A. Encoder Direction & Tick Polarity Test
1. Connect via serial terminal at 57600 baud.
2. Send `r\r` to reset encoders.
3. Manually push the robot forward by hand.
4. Send `e\r`.
5. **Verification**: Both numbers must be **positive**. If the Left or Right count is negative, swap the Phase A and Phase B wires for that encoder.

### B. Speed & Odometry Scale Calibration
- **Wheel Diameter**: Measure physical wheel diameter with calipers (e.g., $65\text{ mm} = 0.065\text{ m}$).
- **Encoder Resolution**: Jack the wheels up and rotate exactly 1 full 360° turn by hand. The delta encoder value reported by `e` is your `encoder_resolution` (e.g., `4000` ticks/rev).
- **Track Width**: Place the robot on the floor, command a 360° in-place spin (`ros2 topic pub /cmd_vel ... angular.z: 0.5`). If the reported `/odom` rotation overshoots or undershoots 360°, tune `wheel_track` in `arduino_params.yaml`.

### C. Traction Control Tuning (in `ROSArduinoBridge.ino`)
- `TCS_SLIP_TOLERANCE` (default `4`): Allowable wheel speed discrepancy above commanded rate before torque reduction is triggered.
- `TCS_MAX_ACCEL` (default `8`): Max physical wheel acceleration per frame before runaway protection engages.
- `TCS_DIFF_SLIP_THRESHOLD` (default `5`): Wheel speed difference threshold for straight-line differential locking.
