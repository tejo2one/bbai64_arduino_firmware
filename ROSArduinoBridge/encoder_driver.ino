/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;
          	
	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return -left_enc_pos; // Inverted so forward rotation is positive on both wheels
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#elif defined(ARDUINO_EVERY_ENC_COUNTER)


volatile long encLeftValue;
volatile long encRightValue;

//                  00, 01, 10, 11  -> New state
int8_t lookup[] =  { 0,  1, -1,  0   // Last state 00
                   ,-1,  0,  0,  1   // Last state 01
                   , 1,  0,  0, -1   // Last state 10
                   , 0, -1,  1,  0}; // Last state 11
                   

void encLeftInterrupt() {
  static char last=0;
  char curr = 0;
  int idx=0;

  if (digitalRead(encLeftAPin)) curr |= 0b10;
  if (digitalRead(encLeftBPin)) curr |= 0b01;

  idx = (last << 2) | curr;
  last = curr;

  encLeftValue += lookup[idx];
}



void encRightInterrupt() {
  static char last=0;
  char curr = 0;
  int idx =0;
  
  if (digitalRead(encRightAPin)) curr |= 0b10;
  if (digitalRead(encRightBPin)) curr |= 0b01;

  idx = (last << 2) | curr;
  last = curr;

  encRightValue += lookup[idx];
}


long readEncoder(int i) {
if (i == LEFT) return encLeftValue;
else return encRightValue;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
if (i == LEFT){
  encLeftValue=0L;
  return;
} else { 
  encRightValue=0L;
  return;
}
}
#elif defined(NO_ENCODERS)
long readEncoder(int i) {
  return 0L;
}

void resetEncoder(int i) {
  // No-op for open-loop operation
}
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

