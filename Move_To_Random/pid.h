#ifndef _PID_h
#define _PID_h
#include <stdint.h>
#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)


class PID {
  public:

    PID(float P, float I, float D);                 // Constructor, not order of P I & D arguments when calling.
    void setGains(float P, float I, float D);       // This function updates the values of the gains
    void reset();                                   // Useful to remove any intergral wind-up
    float update(float demand, float measurement);  // This update takes a demand and measurement.
    float update(float measurement);                // This update assumes the same demand as last set.

    void print_components(); //This function prints the individual components of the control signal and can be used for debugging

    float Kp_output = 0;
    float Ki_output = 0;
    float Kd_output = 0;

    /* Private functions and variables are defined here. These functions / variables cannot be accessed from outside the class.
       For example, if we try to set the value of Kp in the file "Romi.h", we will get an error (Try it out!)
    */
  private:

    //Control gains
    float Kp; //Proportional
    float Ki; //Integral
    float Kd; //Derivative


    //Values to store
    float output_signal       = 0;
    float last_demand         = 0;
    float last_error          = 0; //For calculating the derivative term
    float integral_error      = 0; //For storing the integral of the error
    unsigned long last_millis = 0;

};

/*
   Class constructor
   This runs whenever we create an instance of the class
*/
PID::PID(float P, float I, float D)
{
  //Store the gains
  setGains(P, I, D);
  //Set last_millis
  reset();
}

/*
   This function prints the individual contributions to the total contol signal
   You can call this yourself for debugging purposes, or set the debug flag to true to have it called
   whenever the update function is called.
*/
void PID::print_components() {
  if( SERIAL_ACTIVE ) {
  Serial.print(Kp_output);
  Serial.print(", ");
  Serial.print(Kd_output);
  Serial.print(", ");
  Serial.print(Ki_output);
  Serial.print(", ");
  Serial.println(output_signal);
  }
}


void PID::reset() {
  last_demand     = 0;
  last_error      = 0;
  integral_error  = 0;
  Kp_output       = 0;
  Ki_output       = 0;
  Kd_output       = 0;
  last_millis     = millis();
}

/*
   This function sets the gains of the PID controller
*/
void PID::setGains(float P, float I, float D) {
  Kp = P;
  Kd = D;
  Ki = I;
}

/*
   This is the update function.
   This function should be called repeatedly.
   It takes a measurement of a particular quantity and a desired value for that quantity as input
   It returns an output; this can be sent directly to the motors,
   or perhaps combined with other control outputs
*/
float PID::update(float demand, float measurement) {
  
  //Calculate how much time (in milliseconds) has passed since the last update call
  long time_now = millis();
  long diff_time = time_now - last_millis;
  last_millis = time_now;
  
  float time_delta = (float)diff_time;

  
  // Save the demand passed in as an argument
  // so that we can run a different update function
  // by just using a measurement (demand i.e. stays the same)
  last_demand = demand;

  // Calculate error between demand and measurement.
  float error = demand - measurement;

  //This represents the error derivative
  float error_delta = (last_error - error) / time_delta;
  last_error = error;

  // Integral term.
  integral_error += (error * time_delta);

  //Calculate P,I,D Term contributions.
  Kp_output = Kp * error;
  Kd_output = Kd * error_delta;
  Ki_output = Ki * integral_error;

  //Add the three components to get the total output
  output_signal = Kp_output + Kd_output + Ki_output;

  // Pass the result back.
  return output_signal;
}

float PID::update( float measurement) {
//Calculate how much time (in milliseconds) has passed since the last update call
  long time_now = millis();
  long diff_time = time_now - last_millis;
  last_millis = time_now;
  
  float time_delta = (float)diff_time;
  
  
  // Calculate error between demand and measurement.
  // !!! Note !!!
  // This version of update uses the saved last_demand
  float error = last_demand - measurement;

  //This represents the error derivative
  float error_delta = (last_error - error) / time_delta;
  last_error = error;

  // Integral term.
  integral_error += (error * time_delta);

  //Calculate P,I,D Term contributions.
  Kp_output = Kp * error;
  Kd_output = Kd * error_delta;
  Ki_output = Ki * integral_error;

  //Add the three components to get the total output
  output_signal = Kp_output + Kd_output + Ki_output;

  // Pass the result back.
  return output_signal;
}





#endif
