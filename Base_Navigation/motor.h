#ifndef _Motor_h
#define _Motor_h

// A class to neatly contain commands for the 
// motors, to take care of +/- values, a min/max
// power value, & pin setup.

class Motor {
  public:
    Motor( int pwm, int dir );
    void setPower(int demand);
    void setPower(float demand);
    
  private:
    int pwm_pin;
    int dir_pin;

};

// Constructor: pass in pin for motor.
Motor::Motor( int pwm, int dir ) {
  pwm_pin = pwm;
  dir_pin = dir;

  pinMode(pwm_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);

  digitalWrite(pwm_pin, LOW);
  digitalWrite(dir_pin, LOW);
  
}

// Just typecasts a float to int, calling
// the other function by the same name.
void Motor::setPower(float demand) {
  setPower( (int)demand );
}

// Sets the power of the motor, takes
// care of +/- and min/max value.
void Motor::setPower(int demand) {
  
  //Handle setting directions
  if( demand < 0 ) {    
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }

  // Get rid of - sign.
  demand = abs(demand);

  // Keep between 0 255
  demand = constrain( demand, 0, 255 );
  
  //Write out.
  analogWrite(pwm_pin, demand );
}

#endif

