/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined CYTRON_MOTOR_DRIVER
  void initMotorController() {
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_DIR, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR, OUTPUT);
    
    // Force all pins LOW on startup
    digitalWrite(LEFT_MOTOR_PWM, LOW);
    digitalWrite(LEFT_MOTOR_DIR, LOW);
    digitalWrite(RIGHT_MOTOR_PWM, LOW);
    digitalWrite(RIGHT_MOTOR_DIR, LOW);
    analogWrite(LEFT_MOTOR_PWM, 0);
    analogWrite(RIGHT_MOTOR_PWM, 0);
  }

  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) {
      if (spd > 0) {
        if (spd > MAX_PWM) spd = MAX_PWM;
        digitalWrite(LEFT_MOTOR_DIR, HIGH); // Forward (D6 HIGH)
        analogWrite(LEFT_MOTOR_PWM, spd);   // PWM on D5
      } else if (spd < 0) {
        spd = -spd;
        if (spd > MAX_PWM) spd = MAX_PWM;
        digitalWrite(LEFT_MOTOR_DIR, LOW);  // Reverse (D6 LOW)
        analogWrite(LEFT_MOTOR_PWM, spd);   // PWM on D5
      } else {
        analogWrite(LEFT_MOTOR_PWM, 0);
        digitalWrite(LEFT_MOTOR_PWM, LOW);
        digitalWrite(LEFT_MOTOR_DIR, LOW);
      }
    } else { // RIGHT MOTOR
      if (spd > 0) {
        if (spd > MAX_PWM) spd = MAX_PWM;
        digitalWrite(RIGHT_MOTOR_DIR, LOW);  // Forward (D9 LOW)
        analogWrite(RIGHT_MOTOR_PWM, spd);   // PWM on D10
      } else if (spd < 0) {
        spd = -spd;
        if (spd > MAX_PWM) spd = MAX_PWM;
        digitalWrite(RIGHT_MOTOR_DIR, HIGH); // Reverse (D9 HIGH)
        analogWrite(RIGHT_MOTOR_PWM, spd);   // PWM on D10
      } else {
        analogWrite(RIGHT_MOTOR_PWM, 0);
        digitalWrite(RIGHT_MOTOR_PWM, LOW);
        digitalWrite(RIGHT_MOTOR_DIR, LOW);
      }
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    // Set all control pins to OUTPUT
    pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);

    // Initial safe stop on boot
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_ENABLE, 0);
    digitalWrite(LEFT_MOTOR_ENABLE, LOW);

    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_ENABLE, 0);
    digitalWrite(RIGHT_MOTOR_ENABLE, LOW);
  }
  
  void setMotorSpeed(int i, int spd) {
    int in1Pin, in2Pin, pwmPin;

    if (i == LEFT) {
      in1Pin = LEFT_MOTOR_IN1;    // Pin 6 (IN1)
      in2Pin = LEFT_MOTOR_IN2;    // Pin 7 (IN2)
      pwmPin = LEFT_MOTOR_ENABLE; // Pin 5 (ENA)
    } else {
      in1Pin = RIGHT_MOTOR_IN1;    // Pin 8 (IN3)
      in2Pin = RIGHT_MOTOR_IN2;    // Pin 9 (IN4)
      pwmPin = RIGHT_MOTOR_ENABLE; // Pin 10 (ENB)
    }

    if (spd > 0) {
      if (spd > MAX_PWM) spd = MAX_PWM;
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, spd);
    } else if (spd < 0) {
      spd = -spd;
      if (spd > MAX_PWM) spd = MAX_PWM;
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      analogWrite(pwmPin, spd);
    } else {
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, LOW);
      analogWrite(pwmPin, 0);
      digitalWrite(pwmPin, LOW);
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
