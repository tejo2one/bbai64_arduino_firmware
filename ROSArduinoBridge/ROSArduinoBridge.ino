/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define USE_BASE  // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
/* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019

/* The Pololu MC33926 dual motor driver shield */
//#define POLOLU_MC33926

/* The RoboGaia encoder shield */
//#define ROBOGAIA

/* Encoders directly attached to Arduino board */
#define ARDUINO_ENC_COUNTER
//#define ARDUINO_EVERY_ENC_COUNTER
//#define NO_ENCODERS

/* Motor driver selection */
#define CYTRON_MOTOR_DRIVER
//#define L298_MOTOR_DRIVER
#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE 57600

/* Maximum PWM signal - Cap increased to 220 (~86% duty cycle for higher torque) */
#define MAX_PWM 220

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"
#endif

#ifdef USE_BASE
/* Motor driver function definitions */
#include "motor_driver.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE 30  // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
#define AUTO_STOP_INTERVAL 1000
long lastMotorCommand = 0;

/* Closed-Loop Speed Feedback Controller Configuration */
#define FEEDBACK_RATE 30             // 30 Hz feedback loop (PID / Speed regulation)
const int FEEDBACK_INTERVAL = 1000 / FEEDBACK_RATE;
unsigned long nextFeedback = FEEDBACK_INTERVAL;

// Calibrated max speed in ticks per 33ms frame (~450 ticks/sec at max PWM)
#define MAX_TICKS_PER_FRAME 15

// Closed-loop gain parameters
#define FB_KP 2                      // Proportional boost gain
#define FB_KI_STEP 1                 // Integral step increase per frame when speed is below target

// Soft-Start / Slew Rate Limiter parameters
#define MIN_BASE_PWM 35              // Smooth Base Starting PWM (~14% duty cycle) for gentle breakaway
#define PWM_RAMP_STEP 15             // Slew rate per 30Hz frame for smooth acceleration

// Feedback state variables
long lastFeedbackEncLeft = 0;
long lastFeedbackEncRight = 0;
long measuredSpeedLeft = 0;
long measuredSpeedRight = 0;

int targetTicksLeft = 0;
int targetTicksRight = 0;
int feedbackPwmLeft = 0;
int feedbackPwmRight = 0;

int targetLeftPwm = 0;
int targetRightPwm = 0;
int currentLeftPwm = 0;
int currentRightPwm = 0;

/* =========================================================================
   TRACTION CONTROL SYSTEM (TCS) & ELECTRONIC LIMITED-SLIP DIFFERENTIAL
   ========================================================================= */
#define TCS_ENABLED 1
#define TCS_SLIP_TOLERANCE 4        // Allowable excess speed above target (ticks/frame)
#define TCS_MAX_ACCEL 8             // Max physical wheel acceleration (ticks/frame^2)
#define TCS_DIFF_SLIP_THRESHOLD 5   // Max allowable speed divergence during straight travel
#define TCS_TORQUE_CUT 18           // PWM torque reduction applied per frame when slip is detected

// TCS Real-time State Flags
bool tcsActive = false;
bool tcsSlipLeft = false;
bool tcsSlipRight = false;
bool tcsRunawayGuard = false;

long prevSpeedLeft = 0;
long prevSpeedRight = 0;

int applyDeadband(int pwm) {
  if (pwm == 0) return 0;
  int sign = (pwm > 0) ? 1 : -1;
  int mag = abs(pwm);
  if (mag > 255) mag = 255;
  // Rescale incoming range [1 .. 255] to [MIN_BASE_PWM .. MAX_PWM] (e.g., 42 .. 191)
  int rescaled = MIN_BASE_PWM + ((long)mag * (MAX_PWM - MIN_BASE_PWM)) / 255;
  if (rescaled > MAX_PWM) rescaled = MAX_PWM;
  return sign * rescaled;
}

int stepRamp(int current, int target, int step) {
  if (target == 0) {
    if (abs(current) <= MIN_BASE_PWM || abs(current) <= step) {
      return 0;
    }
    if (current > 0) current -= step;
    else current += step;
    return current;
  }

  if (current == 0) {
    int sign = (target > 0) ? 1 : -1;
    int base = (abs(target) < MIN_BASE_PWM) ? abs(target) : MIN_BASE_PWM;
    return sign * base;
  }

  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

void resetFeedback() {
  lastFeedbackEncLeft = readEncoder(LEFT);
  lastFeedbackEncRight = readEncoder(RIGHT);
  measuredSpeedLeft = 0;
  measuredSpeedRight = 0;
  prevSpeedLeft = 0;
  prevSpeedRight = 0;
  targetTicksLeft = 0;
  targetTicksRight = 0;
  feedbackPwmLeft = 0;
  feedbackPwmRight = 0;
  targetLeftPwm = 0;
  targetRightPwm = 0;
  tcsActive = false;
  tcsSlipLeft = false;
  tcsSlipRight = false;
  tcsRunawayGuard = false;
}

void updateFeedbackController() {
  // Read current encoder positions
  long curL = readEncoder(LEFT);
  long curR = readEncoder(RIGHT);

  measuredSpeedLeft = curL - lastFeedbackEncLeft;
  measuredSpeedRight = curR - lastFeedbackEncRight;
  lastFeedbackEncLeft = curL;
  lastFeedbackEncRight = curR;

  // If stopped, decay outputs to 0
  if (targetTicksLeft == 0 && targetTicksRight == 0 && targetLeftPwm == 0 && targetRightPwm == 0) {
    feedbackPwmLeft = 0;
    feedbackPwmRight = 0;
    prevSpeedLeft = 0;
    prevSpeedRight = 0;
    tcsActive = false;
    tcsSlipLeft = false;
    tcsSlipRight = false;
    tcsRunawayGuard = false;
    currentLeftPwm = stepRamp(currentLeftPwm, 0, PWM_RAMP_STEP);
    currentRightPwm = stepRamp(currentRightPwm, 0, PWM_RAMP_STEP);
    setMotorSpeeds(currentLeftPwm, currentRightPwm);
    return;
  }

  // Open-Loop Direct PWM Mode (Web Cockpit / Teleoperation 'o' command)
  if (targetTicksLeft == 0 && targetTicksRight == 0) {
    feedbackPwmLeft = applyDeadband(targetLeftPwm);
    feedbackPwmRight = applyDeadband(targetRightPwm);
    currentLeftPwm = stepRamp(currentLeftPwm, feedbackPwmLeft, PWM_RAMP_STEP);
    currentRightPwm = stepRamp(currentRightPwm, feedbackPwmRight, PWM_RAMP_STEP);
    setMotorSpeeds(currentLeftPwm, currentRightPwm);
    return;
  }

  // -------------------------------------------------------------------------
  // 1. TRACTION CONTROL SYSTEM (TCS) - SLIP & RUNAWAY EVALUATION
  // -------------------------------------------------------------------------
  tcsSlipLeft = false;
  tcsSlipRight = false;
  tcsRunawayGuard = false;

  // Calculate Wheel Acceleration (ticks / frame^2)
  long accelL = abs(measuredSpeedLeft - prevSpeedLeft);
  long accelR = abs(measuredSpeedRight - prevSpeedRight);
  prevSpeedLeft = measuredSpeedLeft;
  prevSpeedRight = measuredSpeedRight;

  // A. Over-speed Wheel Spin Detection (Anti-Spin)
  if (targetTicksLeft != 0) {
    if (abs(measuredSpeedLeft) > (abs(targetTicksLeft) + TCS_SLIP_TOLERANCE) || accelL > TCS_MAX_ACCEL) {
      tcsSlipLeft = true;
    }
  }
  if (targetTicksRight != 0) {
    if (abs(measuredSpeedRight) > (abs(targetTicksRight) + TCS_SLIP_TOLERANCE) || accelR > TCS_MAX_ACCEL) {
      tcsSlipRight = true;
    }
  }

  // B. Direction Inconsistency Guard (Anti-Runaway / Spinout Prevention)
  // If wheel rotates against commanded velocity vector, clamp output to prevent positive-feedback spinout
  bool dirFaultL = false;
  bool dirFaultR = false;
  if (targetTicksLeft > 0 && measuredSpeedLeft < -2) dirFaultL = true;
  else if (targetTicksLeft < 0 && measuredSpeedLeft > 2) dirFaultL = true;

  if (targetTicksRight > 0 && measuredSpeedRight < -2) dirFaultR = true;
  else if (targetTicksRight < 0 && measuredSpeedRight > 2) dirFaultR = true;

  if (dirFaultL || dirFaultR) {
    tcsRunawayGuard = true;
  }

  // C. Electronic Limited-Slip Differential (e-LSD) for Straight Travel
  if (targetTicksLeft != 0 && targetTicksRight != 0 && targetTicksLeft == targetTicksRight) {
    long speedDiff = abs(measuredSpeedLeft - measuredSpeedRight);
    if (speedDiff > TCS_DIFF_SLIP_THRESHOLD) {
      if (abs(measuredSpeedLeft) > abs(measuredSpeedRight)) {
        tcsSlipLeft = true;
      } else {
        tcsSlipRight = true;
      }
    }
  }

  tcsActive = (tcsSlipLeft || tcsSlipRight || tcsRunawayGuard);

  // -------------------------------------------------------------------------
  // 2. COMPUTE BASE SPEED ERRORS
  // -------------------------------------------------------------------------
  long errorL = targetTicksLeft - measuredSpeedLeft;
  long errorR = targetTicksRight - measuredSpeedRight;

  // Cross-Coupled Wheel Synchronization (Electronic Differential Lock)
  // ONLY active when driving straight AND neither wheel has direction fault
  if (targetTicksLeft != 0 && targetTicksRight != 0 && targetTicksLeft == targetTicksRight && !tcsRunawayGuard) {
    // Only lock when both wheels are moving in the intended direction
    bool leftDirOk = (targetTicksLeft > 0) ? (measuredSpeedLeft >= 0) : (measuredSpeedLeft <= 0);
    bool rightDirOk = (targetTicksRight > 0) ? (measuredSpeedRight >= 0) : (measuredSpeedRight <= 0);
    if (leftDirOk && rightDirOk) {
      long actualDiff = measuredSpeedLeft - measuredSpeedRight;
      long syncError = actualDiff; // expectedDiff is 0 because targetTicksLeft == targetTicksRight
      if (targetTicksLeft < 0) syncError = -syncError; // Invert in reverse

      errorL -= (syncError * 2); // Throttles back faster wheel
      errorR += (syncError * 2); // Boosts lagging wheel
    }
  }

  // -------------------------------------------------------------------------
  // 3. CLOSED-LOOP LEFT MOTOR FEEDBACK WITH TCS REGULATION
  // -------------------------------------------------------------------------
  if (targetTicksLeft != 0) {
    if (dirFaultL) {
      // Direction runaway guard: clamp Left to base PWM
      feedbackPwmLeft = (targetTicksLeft > 0) ? MIN_BASE_PWM : -MIN_BASE_PWM;
    } else if (targetTicksLeft > 0) { // Forward
      if (measuredSpeedLeft == 0 && errorL > 0) {
        feedbackPwmLeft += FB_KI_STEP;
      } else if (errorL > 0) {
        feedbackPwmLeft += (errorL * FB_KP + FB_KI_STEP);
      } else if (errorL < 0) {
        feedbackPwmLeft -= (abs(errorL) * FB_KP + FB_KI_STEP);
      }
      // TCS Anti-Spin Torque Cut
      if (tcsSlipLeft && feedbackPwmLeft > MIN_BASE_PWM) {
        feedbackPwmLeft -= TCS_TORQUE_CUT;
        if (feedbackPwmLeft < MIN_BASE_PWM) feedbackPwmLeft = MIN_BASE_PWM;
      }
      if (feedbackPwmLeft < MIN_BASE_PWM) feedbackPwmLeft = MIN_BASE_PWM;
      if (feedbackPwmLeft > MAX_PWM) feedbackPwmLeft = MAX_PWM;
    } else { // Reverse
      if (measuredSpeedLeft == 0 && errorL < 0) {
        feedbackPwmLeft -= FB_KI_STEP;
      } else if (errorL < 0) {
        feedbackPwmLeft -= (abs(errorL) * FB_KP + FB_KI_STEP);
      } else if (errorL > 0) {
        feedbackPwmLeft += (errorL * FB_KP + FB_KI_STEP);
      }
      // TCS Anti-Spin Torque Cut
      if (tcsSlipLeft && feedbackPwmLeft < -MIN_BASE_PWM) {
        feedbackPwmLeft += TCS_TORQUE_CUT;
        if (feedbackPwmLeft > -MIN_BASE_PWM) feedbackPwmLeft = -MIN_BASE_PWM;
      }
      if (feedbackPwmLeft > -MIN_BASE_PWM) feedbackPwmLeft = -MIN_BASE_PWM;
      if (feedbackPwmLeft < -MAX_PWM) feedbackPwmLeft = -MAX_PWM;
    }
  } else {
    feedbackPwmLeft = 0;
  }

  // -------------------------------------------------------------------------
  // 4. CLOSED-LOOP RIGHT MOTOR FEEDBACK WITH TCS REGULATION
  // -------------------------------------------------------------------------
  if (targetTicksRight != 0) {
    if (dirFaultR) {
      // Direction runaway guard: clamp Right to base PWM
      feedbackPwmRight = (targetTicksRight > 0) ? MIN_BASE_PWM : -MIN_BASE_PWM;
    } else if (targetTicksRight > 0) { // Forward
      if (measuredSpeedRight == 0 && errorR > 0) {
        feedbackPwmRight += FB_KI_STEP;
      } else if (errorR > 0) {
        feedbackPwmRight += (errorR * FB_KP + FB_KI_STEP);
      } else if (errorR < 0) {
        feedbackPwmRight -= (abs(errorR) * FB_KP + FB_KI_STEP);
      }
      // TCS Anti-Spin Torque Cut
      if (tcsSlipRight && feedbackPwmRight > MIN_BASE_PWM) {
        feedbackPwmRight -= TCS_TORQUE_CUT;
        if (feedbackPwmRight < MIN_BASE_PWM) feedbackPwmRight = MIN_BASE_PWM;
      }
      if (feedbackPwmRight < MIN_BASE_PWM) feedbackPwmRight = MIN_BASE_PWM;
      if (feedbackPwmRight > MAX_PWM) feedbackPwmRight = MAX_PWM;
    } else { // Reverse
      if (measuredSpeedRight == 0 && errorR < 0) {
        feedbackPwmRight -= FB_KI_STEP;
      } else if (errorR < 0) {
        feedbackPwmRight -= (abs(errorR) * FB_KP + FB_KI_STEP);
      } else if (errorR > 0) {
        feedbackPwmRight += (errorR * FB_KP + FB_KI_STEP);
      }
      // TCS Anti-Spin Torque Cut
      if (tcsSlipRight && feedbackPwmRight < -MIN_BASE_PWM) {
        feedbackPwmRight += TCS_TORQUE_CUT;
        if (feedbackPwmRight > -MIN_BASE_PWM) feedbackPwmRight = -MIN_BASE_PWM;
      }
      if (feedbackPwmRight > -MIN_BASE_PWM) feedbackPwmRight = -MIN_BASE_PWM;
      if (feedbackPwmRight < -MAX_PWM) feedbackPwmRight = -MAX_PWM;
    }
  } else {
    feedbackPwmRight = 0;
  }

  // Smooth ramp transition toward feedback-regulated PWM
  currentLeftPwm = stepRamp(currentLeftPwm, feedbackPwmLeft, PWM_RAMP_STEP);
  currentRightPwm = stepRamp(currentRightPwm, feedbackPwmRight, PWM_RAMP_STEP);
  setMotorSpeeds(currentLeftPwm, currentRightPwm);
}
#endif

/* Variable initialization */

// Heartbeat LED variables
#define STATUS_LED 13
unsigned long lastBlinkTime = 0;
const int blinkInterval = 500; // Blink every 500ms (1 Hz)
int ledState = LOW;

// A pair of varibles to help parse serial commands...

int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case PING:
      Serial.println(Ping(arg1));
      break;
#ifdef USE_SERVOS
    case SERVO_WRITE:
      servos[arg1].setTargetPosition(arg2);
      Serial.println("OK");
      break;
    case SERVO_READ:
      Serial.println(servos[arg1].getServo().read());
      break;
#endif

#ifdef USE_BASE
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetFeedback();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        targetTicksLeft = 0;
        targetTicksRight = 0;
        targetLeftPwm = 0;
        targetRightPwm = 0;
        feedbackPwmLeft = 0;
        feedbackPwmRight = 0;
        moving = 0;
        resetPID();
      } else {
        moving = 1;
        targetTicksLeft = arg1;
        targetTicksRight = arg2;
        if (feedbackPwmLeft == 0 && arg1 != 0) {
          feedbackPwmLeft = (arg1 > 0) ? MIN_BASE_PWM : -MIN_BASE_PWM;
        }
        if (feedbackPwmRight == 0 && arg2 != 0) {
          feedbackPwmRight = (arg2 > 0) ? MIN_BASE_PWM : -MIN_BASE_PWM;
        }
      }
      Serial.println("OK");
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      moving = 0;
      targetLeftPwm = arg1;
      targetRightPwm = arg2;
      targetTicksLeft = 0;
      targetTicksRight = 0;
      if (arg1 == 0 && arg2 == 0) {
        feedbackPwmLeft = 0;
        feedbackPwmRight = 0;
      } else {
        feedbackPwmLeft = applyDeadband(arg1);
        feedbackPwmRight = applyDeadband(arg2);
      }
      Serial.println("OK");
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial.println("OK");
      break;
    case READ_MOTOR_PINS:
      #if defined(L298_MOTOR_DRIVER)
      Serial.print("L298N LEFT: ENA(D5)=");
      Serial.print(OCR0B);
      Serial.print(" IN1(D6)=");
      Serial.print(digitalRead(6));
      Serial.print(" IN2(D7)=");
      Serial.print(digitalRead(7));
      Serial.print(" | RIGHT: ENB(D10)=");
      Serial.print(OCR1B);
      Serial.print(" IN3(D8)=");
      Serial.print(digitalRead(8));
      Serial.print(" IN4(D9)=");
      Serial.print(digitalRead(9));
      Serial.print(" | TARGET: L=");
      Serial.print(targetLeftPwm);
      Serial.print(" R=");
      Serial.print(targetRightPwm);
      Serial.print(" | RAMPED: L=");
      Serial.print(currentLeftPwm);
      Serial.print(" R=");
      Serial.println(currentRightPwm);
      #elif defined(CYTRON_MOTOR_DRIVER)
      Serial.print("CYTRON LEFT: PWM(D5)=");
      Serial.print(OCR0B);
      Serial.print(" DIR(D6)=");
      Serial.print(digitalRead(6));
      Serial.print(" CUR=");
      Serial.print(currentLeftPwm);
      Serial.print(" TARG=");
      Serial.print(targetLeftPwm);
      Serial.print(" | RIGHT: D10=");
      Serial.print(OCR1B);
      Serial.print(" D9=");
      Serial.print(OCR1A);
      Serial.print(" CUR=");
      Serial.print(currentRightPwm);
      Serial.print(" TARG=");
      Serial.print(targetRightPwm);
      Serial.print(" | Encoders: L=");
      Serial.print(readEncoder(LEFT));
      Serial.print(" R=");
      Serial.print(readEncoder(RIGHT));
      Serial.print(" | TCS: ACT=");
      Serial.print(tcsActive ? 1 : 0);
      Serial.print(" SLIP_L=");
      Serial.print(tcsSlipLeft ? 1 : 0);
      Serial.print(" SLIP_R=");
      Serial.print(tcsSlipRight ? 1 : 0);
      Serial.print(" GUARD=");
      Serial.println(tcsRunawayGuard ? 1 : 0);
      #endif
      break;
#endif
    default:
      Serial.println("Invalid Command");
      break;
  }
}
// #ifdef ARDUINO_EVERY_ENC_COUNTER
// const byte encLeftAPin = LEFT_ENC_PIN_A;
// const byte encLeftBPin = LEFT_ENC_PIN_B;
// const byte encRightAPin = RIGHT_ENC_PIN_A;
// const byte encRightBPin = RIGHT_ENC_PIN_B;

// extern void encLeftInterrupt();
// extern void encRightInterrupt();
// #endif

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
  // Initialize Status LED
  pinMode(STATUS_LED, OUTPUT);

  Serial.println("WelCome!!!");
// Initialize the motor controller if used */
#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER
  //set as inputs
  DDRD &= ~(1 << LEFT_ENC_PIN_A);
  DDRD &= ~(1 << LEFT_ENC_PIN_B);
  DDRC &= ~(1 << RIGHT_ENC_PIN_A);
  DDRC &= ~(1 << RIGHT_ENC_PIN_B);

  //enable pull up resistors
  PORTD |= (1 << LEFT_ENC_PIN_A);
  PORTD |= (1 << LEFT_ENC_PIN_B);
  PORTC |= (1 << RIGHT_ENC_PIN_A);
  PORTC |= (1 << RIGHT_ENC_PIN_B);

  // tell pin change mask to listen to left encoder pins
  PCMSK2 |= (1 << LEFT_ENC_PIN_A) | (1 << LEFT_ENC_PIN_B);
  // tell pin change mask to listen to right encoder pins
  PCMSK1 |= (1 << RIGHT_ENC_PIN_A) | (1 << RIGHT_ENC_PIN_B);

  // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  PCICR |= (1 << PCIE1) | (1 << PCIE2);
#elif defined(ARDUINO_EVERY_ENC_COUNTER)
  pinMode(encLeftAPin, INPUT_PULLUP);
  pinMode(encLeftBPin, INPUT_PULLUP);
  pinMode(encRightAPin, INPUT_PULLUP);
  pinMode(encRightBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encLeftAPin), encLeftInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encLeftBPin), encLeftInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRightAPin), encRightInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRightBPin), encRightInterrupt, CHANGE);
#endif
  initMotorController();
  resetFeedback();
  resetPID();
  setMotorSpeeds(0, 0);
  moving = 0;
#endif

  /* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
      servoPins[i],
      stepDelay[i],
      servoInitPosition[i]);
  }
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    } else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      } else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      } else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

// If we are using base control, run a closed-loop feedback speed calculation at 30 Hz
#ifdef USE_BASE
  if (millis() > nextFeedback) {
    updateFeedbackController();
    nextFeedback += FEEDBACK_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    targetTicksLeft = 0;
    targetTicksRight = 0;
    targetLeftPwm = 0;
    targetRightPwm = 0;
    feedbackPwmLeft = 0;
    feedbackPwmRight = 0;
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
// --- HEARTBEAT CODE ---
  if (millis() - lastBlinkTime > blinkInterval) {
    lastBlinkTime = millis();   // Reset timer
    ledState = !ledState;       // Toggle state (High -> Low -> High)
    digitalWrite(STATUS_LED, ledState);
  }
}
