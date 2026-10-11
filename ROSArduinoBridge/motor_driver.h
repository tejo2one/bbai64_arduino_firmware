/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef CYTRON_MOTOR_DRIVER
  // Cytron MDD10A Dual Channel DC Motor Driver
  // Left Motor
  #define LEFT_MOTOR_PWM  5   // PWM 1 (D5)
  #define LEFT_MOTOR_DIR  6   // DIR 1 (D6)
  
  // Right Motor
  #define RIGHT_MOTOR_PWM 10  // PWM 2 (D10)
  #define RIGHT_MOTOR_DIR 9   // DIR 2 (D9)
#elif defined L298_MOTOR_DRIVER
// Left Motor
  #define LEFT_MOTOR_ENABLE 5   // PWM Pin (ENA)
  #define LEFT_MOTOR_IN1    6   // Direction Pin 1
  #define LEFT_MOTOR_IN2    7   // Direction Pin 2
  
  // Right Motor
  #define RIGHT_MOTOR_ENABLE 10  // PWM Pin (ENB)
  #define RIGHT_MOTOR_IN1    8   // Direction Pin 3
  #define RIGHT_MOTOR_IN2    9   // Direction Pin 4
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
