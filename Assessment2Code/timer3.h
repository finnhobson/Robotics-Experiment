

#ifndef _TIMER3_h
#define _TIMER3_h

// This code has a dependency on encoders.h, using
// the following global variables found there:
// Note: "extern" here means that the compiler will go
// and look for these variables in a different file, 
// instead of creating them here. 
// Therefore, if you rename them or remove them from
// enconders.h, this code will no longer compile.
extern volatile long e0_count;
extern volatile long e0_count_prior;
extern volatile long e1_count;
extern volatile long e1_count_prior;

// This code has a dependency from the main code, using
// the following global variables found there:
extern volatile float l_speed_t3, r_speed_t3;

// Routine to setupt timer3 to run 
void setupTimer3() {
  
  // disable global interrupts
  cli();          

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual. 
  TCCR3B = TCCR3B | (1 << CS32);
  
  
  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (e.g., for 2hz).
  OCR3A = 625;// 100hz
  //OCR3A = 1249;//50hz
  
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei(); 
  
}

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the 
// compiler.  It automatically associates with Timer3 in
// CTC mode.
//
// Note, if we know Timer3 is firing at 100hz, it seems
// redudant to compute diff/time when time change is a
// constant.
//
ISR( TIMER3_COMPA_vect ) {

  // Debugging: Invert LED state.
  //DEBUG_LED_STATE = !DEBUG_LED_STATE;
  //digitalWrite(13, DEBUG_LED_STATE);

  // Trying to avoid an overflow with
  // big numbers here.
  long diff;
  
  diff = e0_count - e0_count_prior;
  e0_count_prior = e0_count;
  
  // Save speed to global variable.
  l_speed_t3 = (float)diff;
  
  
  diff = e1_count - e1_count_prior;
  e1_count_prior = e1_count;
  
  // Save speed to global variable.
  r_speed_t3 = (float)diff;
  
}

#endif
