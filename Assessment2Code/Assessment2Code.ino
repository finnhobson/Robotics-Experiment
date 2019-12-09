


/*
       @@@@@@@@@@@&*           %@@@@@%       @@@@@@@@@    @@@@@@@@@  @@@@@@@@
       @@@@@@@@@@@@@@@     #@@@@@@@@@@@@    @@@@@@@@@@   @@@@@@@@@* @@@@@@@@@
       @@@@@@   @@@@@@   /@@@@@%  .@@@@@@    @@@/@@@@@ @@@@@@@@@@    @@@@@@
      &@@@@@##&@@@@@@   @@@@@@(   @@@@@@@   @@@,.@@@@@@@@,.@@@@@    @@@@@@
      @@@@@@@@@@@@@    &@@@@@@    @@@@@@   @@@@  @@@@@@@  @@@@@    (@@@@@
     @@@@@@  @@@@@@*   @@@@@@    @@@@@@   .@@@   @@@@@#  @@@@@@    @@@@@&
   @@@@@@@@   @@@@@@%  .@@@@@@@@@@@@@    @@@@@%  @@@@  @@@@@@@@  @@@@@@@@
  %@@@@@@@&   @@@@@@     #@@@@@@@@      @@@@@@   @@@   @@@@@@@/ @@@@@@@@%

  Provided by Paul O'Dowd Nov 2019

  Uploading this code, your Romi should:
    - Do nothing until button A or B are pressed.
    - If button A, it will print the last known map.
    - If button B, it will erase the map and begin operation.
    - Randomly switch between driving straight, turning 90* or 0*,
      driving a random walk, following a line, avoid obstacles.
    - Record lines and obstacles into the map as 'L' and 'O'. 
      Note that, if a sensor is not plugged in or correctly, you
      may get these added to the map randomly and see strange 
      behaviours from your Romi.  
  
  You will need to:
    - Read through this code to undestand what is going on.
    - You may wish to comment out code to a bare minimum for your
      task, or as a starting point.
    - Possibly try some of your own code first, to ensure your
      Romi and sensors are working ok.
    - Check where sensors are wired in relative to this code.
      You can find which pins are in use in the #define section
      below.
    - Check PID tuning - especially if your robot is shakey.
    - Complete classes such as irproximity.h, mapping.h
    - Adjust the dimensions of the map to suit your task.
    - Look at the advanced labsheets for some topics.
    - Decide if you want to use any of this code.
    - Begin to build your own system.
*/

/*****************************************************************************
    INCLUDES (global)

*****************************************************************************/
#include "timer3.h"     // setup/isr for timer3 to calculate wheel speed.
#include "encoders.h"   // setup and isr to manage encoders.
#include "kinematics.h" // calculates x,y,theta from encoders.
#include "motor.h"      // handles power and direction for motors.
#include "pid.h"        // PID implementation.
#include "LineSensor.h" // handles all 3 line sensors as a single class.
#include "mapping.h"    // Used to store and read a metric byte map to EEPROM.
#include "utils.h"      // Used to generate random gaussian numbers.
#include "irproximity.h"// Used for the ir distance sensor.

//#include "imu.h"          // Advanced, work through labsheet if you wish to use.
//#include "magnetometer.h" // Advanced, work through labsheet if you wish to use.

#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

/*****************************************************************************
    DEFINITIONS (global)
    Note, pins taken from the pin mapping for Romi available online.
*****************************************************************************/


#define M0_DIR          16  // Motor Pins.
#define M0_PWM          10
#define M1_DIR          15
#define M1_PWM          9

#define L_SENSE_L       A4  // Line sensor pins
#define L_SENSE_C       A3
#define L_SENSE_R       A2

#define BUZZER_PIN      6   // To make the annoying beeping

#define IR_PROX_PIN    A0   // IR Sensor

#define DEBUG_LED      13   // Using the orange LED for debugging

#define BUTTON_A       14   // Push button labelled A on board.
#define BUTTON_B       30   // Push button labelled B on board.

// Behaviour parameters
#define LINE_THRESHOLD        200.00
#define STRAIGHT_FWD_SPEED    10.0
#define LINE_FOLLOW_SPEED     8.0
#define IR_DETECTD_THRESHOLD  80   // a close reading in mm (danger)
#define IR_AVOIDED_THRESHOLD  140   // a distant reading in mm (safe)

// Speed controller for motors.
// Using same gains for left and right.
#define SPD_PGAIN     3.5
#define SPD_IGAIN     0.1
#define SPD_DGAIN     -1.5

// PID controller gains for heading feedback
#define H_PGAIN   3.0
#define H_IGAIN   0.0001
#define H_DGAIN   0.0


/*****************************************************************************
    CLASS INSTANCES (global)
    Please investigate class tabs for methods.
*****************************************************************************/

LineSensor  LineSensor( L_SENSE_L, L_SENSE_C, L_SENSE_R );  // Class to handle all 3 line sensors.
Motor       L_Motor( M0_PWM, M0_DIR );                       // To set left motor power.
Motor       R_Motor( M1_PWM, M1_DIR );                       // To set right motor power.
PID         L_PID( SPD_PGAIN, SPD_IGAIN, SPD_DGAIN );       // Speed control, left.
PID         R_PID( SPD_PGAIN, SPD_IGAIN, SPD_DGAIN );       // Speed control, right.
PID         H_PID( H_PGAIN, H_IGAIN, H_DGAIN );             // Position control, angle.
SharpIR     IRSensor0( IR_PROX_PIN );                       // Get distance to objects (incomplete class)
Kinematics  RomiPose;                                       // Not using ICC.
Mapper      Map;                                            // Default: 25x25 grid, 72mm resolution.


/*****************************************************************************
    OTHER GLOBAL VARIABLES

*****************************************************************************/


unsigned long update_t;   // Used for timing/flow control for main loop()
unsigned long behaviour_t;// Use to track how long a behaviour has run.

bool angle_setup;
float demand_angle;

// used by timer3.h to calculate left and right wheel speed.
volatile float l_speed_t3, r_speed_t3;

// Different states(behaviours) the robot
// can be in.
int STATE;
#define STATE_CALIBRATE       0    // calibrates line sensor
#define STATE_INITIAL         1     // picks a random next state
#define STATE_FOLLOW_LINE     2     // Basic line following.
#define STATE_RANDOM_WALK     3     // Robot will make random turns 6 seconds
#define STATE_DRIVE_STRAIGHT  4     // Robot drives in a straight line 3 seconds
#define STATE_TURN_TO_ZERO    5     // Turns so the robot faces theta = 0
#define STATE_TURN_TO_PIOVER2 6     // Turns so the robot faces theta = PI/2 (90*)
#define STATE_AVOID_OBSTACLE  7
#define STATE_CHANGE_ANGLE    8




/*****************************************************************************
    SETUP
    REQUIRED, RUNS ONCE ON POWER UP.
*****************************************************************************/
void setup() {

  // Misc pin setup not handled by classes.
  pinMode( BUZZER_PIN, OUTPUT );
  pinMode( DEBUG_LED, OUTPUT );

  // Push buttons.  Note that, by doing a
  // digital write, the button has a default
  // (not pressed) value of HIGH.
  pinMode( BUTTON_A, INPUT );
  digitalWrite( BUTTON_A, HIGH );
  pinMode( BUTTON_B, INPUT );
  digitalWrite( BUTTON_B, HIGH );


  // Begin tracking encoder changes.
  setupEncoder0();
  setupEncoder1();

  // Using Timer3 to calcuate wheel speed
  // in the background at 100hz.
  setupTimer3();

  // We set the robot to start kinematics
  // in the centre of the map.
  // See mapping.h for MAP_X/Y definitions.
  RomiPose.setPose( MAP_X / 2, MAP_Y / 2, 0 );

  angle_setup = false;
  demand_angle = 0;
  
  // Start up the serial port.
  Serial.begin(9600);

  // Delay to connect properly.
  delay(1000);

  // Beep so we know if it is reseting.
  beep(); beep(); beep();

  // Print a debug, so we can see a reset on monitor.
  if ( SERIAL_ACTIVE ) Serial.println("***RESET***");

  // This function reads buttons A and B, and will
  // block your Romi from finishing Startup up until
  // a button is pressed.
  // Please look into helper function section below.
  decideStartUpFromButtons();

  // set Initial State, also resets timestamps
  changeState( STATE_CALIBRATE );

}// end of setup, Ready to go!






/*****************************************************************************
    LOOP
    REQUIRED MAIN CODE, CALLED ITERATIVELY BY ARDUINO AFTER SETUP
*****************************************************************************/
void loop() {

  // Always update kinematics
  RomiPose.update( e0_count, e1_count );

  // Runs a behaviour every 50ms, skips otherwise.
  // Therefore, behaviours update 20 times a second
  if (  millis() - update_t > 50 ) {
    update_t = millis();

    //Serial.print( STATE );
    //Serial.print( ", " );
    Serial.print( demand_angle );
    Serial.print( ", " );
    Serial.println( RomiPose.theta );

    // We check for a line, and if we find one
    // we immediately change state to line following.
    if ( LineSensor.onLine( LINE_THRESHOLD ) ) {

      // Record that we found a line.
      Map.updateMapFeature( 'L' , RomiPose.x, RomiPose.y );

      // Set next state to line following, caught
      // by switch below
      //changeState( STATE_FOLLOW_LINE );

    }

    // We always check for obstacles.  If we
    // detect one, we immediately change state
    // to avoid obstacles. Note, comes after
    // line detection, so in effect higher
    // priority ( the last to change the state)
    //if ( SERIAL_ACTIVE ) Serial.println( IRSensor0.getDistanceInMM() );
    if ( IRSensor0.getDistanceInMM() < IR_DETECTD_THRESHOLD ) {

      // Record that we found an obstruction
      // Note that, this places the object in the map at
      // the romi x/y, not taking account for the distance
      // measure to the object itself.
      // Using LED just to see if it is working.
      digitalWrite(DEBUG_LED, HIGH);
      Map.updateMapFeature( 'O' , RomiPose.x, RomiPose.y );

      // Set next state to obstacle avoidance,
      // caught be switch below.
      changeState( STATE_AVOID_OBSTACLE );

    } else {
      digitalWrite(DEBUG_LED, LOW);
    }


    // Note that, STATE is set at the transition (exit)
    // out of any of the below STATE_ functions, with the 
    // exception of finding the line or an obstacle above.
    // You should rebuild this state machine to suit your
    // objective.  You can do this by changing which state
    // is set when a behaviour finishes (see behaviours code).
    switch ( STATE ) {
      
      case STATE_CALIBRATE:
        calibrateSensors();
        break;

      case STATE_INITIAL:
        initialBehaviour();
        break;

      case STATE_FOLLOW_LINE:
        followLine();
        break;

      case STATE_RANDOM_WALK:
        randomWalk();
        break;

      case STATE_DRIVE_STRAIGHT:
        driveStraight();
        break;

      case STATE_TURN_TO_ZERO:
        turnToThetaZero( );
        break;

      case STATE_TURN_TO_PIOVER2:
        turnToThetaPIOver2( );
        break;

      case STATE_AVOID_OBSTACLE:
        avoidObstacle();
        break;

      case STATE_CHANGE_ANGLE:
        changeAngle();
        break;

      default: // unknown, this would be an error.
        reportUnknownState();
        break;

    } // End of state machine switch()
  } // End of update_t if()

  // Small delay to prevent millis = 0
  delay(1);

}// End of Loop()



void driveStraight() {

  if ( !angle_setup ) {
    demand_angle = RomiPose.theta;
    angle_setup = true;
  }

  // If we have been doing this for more than 3 seconds,
  // we transition out of this behaviour.
  // Note that, behaviour_t is reset by the changeState
  // function, called by every state when transitioning.
  unsigned long elapsed_t = (millis() - behaviour_t );
  if (  elapsed_t > 500 ) {   

    // This ensures that the PID are reset
    // and sets the new STATE flag.
    if ( RomiPose.x <= 0 || RomiPose.y <= 0 || RomiPose.x >= MAP_X || RomiPose.y >= MAP_Y ) {
      changeState( STATE_CHANGE_ANGLE );
    }

  } else { // Otherwise, drive straight.

    // We want to keep the difference in the Romi theta
    // between time steps to 0.
    float delta_theta = RomiPose.theta - demand_angle;


    // Demand 0, change in theta is measurement.
    float bearing = H_PID.update( 0, delta_theta );

    // Foward speed.
    float fwd_bias = STRAIGHT_FWD_SPEED;

    // PID speed control.
    float l_pwr = L_PID.update( (fwd_bias - bearing), l_speed_t3 );
    float r_pwr = R_PID.update( (fwd_bias + bearing), r_speed_t3 );

    // Write power to motors.
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);

  }
}



void changeAngle() {

  if ( !angle_setup ) {
    demand_angle = RomiPose.theta - PI;
    angle_setup = true;
  }

  // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
  // Some crazy atan2 magic.
  // Treats the difference in angle as cartesian x,y components.
  // Cos and Sin are effectively wrapping the values between -1, +1, with a 90 degree phase.
  // So we can pass in values larger/smaller than 0:TWO_PI fine.
  // atan2 returns -PI/+PI, giving us an indication of direction to turn.
  // Between -PI/+PI also means we avoid an extreme demand sent to the heading PID.
  float diff = atan2( sin( ( demand_angle - RomiPose.theta) ), cos( (demand_angle - RomiPose.theta) ) );

  // If we have got the Romi theta to roughly match
  // the demand (by getting the difference to 0(ish)
  // We transition out of this behaviour.
  if ( abs( diff ) < 0.03 ) {

    // This ensures that the PID are reset
    // and sets the new STATE flag.
    changeState( STATE_INITIAL  );

  } else {    // else, turning behaviour

    // Measurement is the different in angle, demand is 0
    // bearing steers us to minimise difference toward 0
    float bearing = H_PID.update( 0, diff );

    // Append to motor speed control
    float l_pwr = L_PID.update( (0 - bearing), l_speed_t3 );
    float r_pwr = R_PID.update( (0 + bearing), r_speed_t3 );

    // Set motor power.
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);

  } // end of abs(diff)<0.03 if()

}// end of behaviour




/*****************************************************************************
    Helper functions.
    These are used to perform some operations which are not a 
    significant part of the Romi behaviour or state machine.


*****************************************************************************/

// Setup helper function to take user input and initiate
// start up mode.
void decideStartUpFromButtons() {

  // Blocks until either button a or b is
  // pressed.
  // You may wish to improve this code to make
  // the user input more robust.
  int mode = -1;
  do {

    if ( SERIAL_ACTIVE ) Serial.println("Waiting for button a (print map) or b (erase map)");

    int btn_a = digitalRead( BUTTON_A );
    int btn_b = digitalRead( BUTTON_B );

    // Decide if we are going to print
    // or erase the map.
    if ( btn_a == LOW ) {
      mode = 0;
    } else if ( btn_b == LOW ) {
      mode = 1;
    }

  } while ( mode < 0 );

  // Acknowledge button press.
  beep();

  if ( mode == 0 ) {  // Print map

    // Because 1 will always be true, you Romi
    // will no be stuck in this loop forever.
    while ( 1 ) {
      Map.printMap();
      delay(2000);
    }

  }

  if ( SERIAL_ACTIVE ) Serial.println("Erasing Map, activating Romi");

  Map.resetMap();

}

// Note, this blocks the flow/timing
// of your code.  Use sparingly.
void beep() {
  analogWrite(6, 1);
  delay(50);
  analogWrite(6, 0);
  delay(50);
}

void reportUnknownState() {
  if ( SERIAL_ACTIVE ) {
    Serial.print("Unknown state: ");
    Serial.println( STATE );
  }
}


/*****************************************************************************
    BEHAVIOURS
    An assortment of behaviours called variously by state machine from Loop.
    You can take inspiration from these to write your own behaviours.
    You will need to build up your own state machine.
    Feel free to start from scratch - you don't have to use this example.
*****************************************************************************/

// The state transition behaviour.
// Resets all the PID, and also beeps so we know it happened.
// Every other behaviour calls this when exiting/transitioning.
// Otherwise, you'll likely see integral wind-up in the PID.
void changeState( int which ) {

  // If, for some reason, we ask to change
  // to the state we are already in, we just
  // return with no action.
  if ( which == STATE ) return;

  // Stop motors.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );

  angle_setup = false;

  // A short beep if debugging
  beep();

  // Reset the timestamp to track how
  // long we exist in the next state (behaviour)
  behaviour_t = millis();

  // If we are changing state, we reset update_t
  // to force an elapsed time before behaviour is
  // actioned (important to stop divide by 0 error.
  update_t = millis();

  // Set the new state to the one requested.
  STATE = which;

  // reset PID
  L_PID.reset();
  R_PID.reset();
  H_PID.reset();

  return;
}

void calibrateSensors() {

  // Make sure motors are off so the robot
  // stays still.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );


  // Line sensor.
  LineSensor.calibrate();

  // Other sensors..?

  // After calibrating, we send the robot to
  // its initial state.
  changeState( STATE_INITIAL );
}


// No initial behaviour for this example.
// You might want to setup a proper initial behaviour
// routine.
// But we'll use it to instead decide a
// random next state.
void initialBehaviour() {

  // Note: we don't select 0 as this is CALIBRATE
  //       we don't select 1 as this is INITIAL (this function)
  //       and random is returning up to 8 (e.g., 7.9) which
  //       is rounded down by (int) to just 7
  //int which = (int)random( 2, 8 );
  int which = 4;

  // Action state transition
  changeState( which );
}


void avoidObstacle() {


  // Get a distance measurement to object.
  float distance = IRSensor0.getDistanceInMM();

  // If we are a safe distance away, change state.
  if (  distance > IR_AVOIDED_THRESHOLD ) {

    // This ensures that the PID are reset
    // and sets the new STATE flag.
    changeState( STATE_INITIAL );
    return;

  } else { // Otherwise, Turn away from obstacle.

    // Using straight foward speed.
    float turn = STRAIGHT_FWD_SPEED;

    // PID speed control, but we just turn on the spot
    float l_pwr = L_PID.update( (0 - turn), l_speed_t3 );
    float r_pwr = R_PID.update( (0 + turn), r_speed_t3 );

    // Write power to motors.
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);

  }
}

void followLine() {


  // If we have lost the line, we change state.
  if ( LineSensor.onLine( LINE_THRESHOLD ) == false ) {

    // This ensures that the PID are reset
    // and sets the new STATE flag.
    changeState( STATE_INITIAL  );

  } else {    // else, line following behaviour

    // Measurement is the different in angle, demand is 0
    // bearing steers us to minimise difference toward 0
    float line_heading  = LineSensor.getHeading();
    float steering      = H_PID.update( 0, line_heading );
    float fwd_bias      = LINE_FOLLOW_SPEED;

    // Append to motor speed control
    float l_pwr = L_PID.update( (fwd_bias - steering), l_speed_t3 );
    float r_pwr = R_PID.update( (fwd_bias + steering), r_speed_t3 );

    // Set motor power.
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);

  } // end of oneLine()==false if()

}// end of behaviour

void turnToThetaPIOver2() {

  float demand_angle = PI / 2;

  // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
  // Some crazy atan2 magic.
  // Treats the difference in angle as cartesian x,y components.
  // Cos and Sin are effectively wrapping the values between -1, +1, with a 90 degree phase.
  // So we can pass in values larger/smaller than 0:TWO_PI fine.
  // atan2 returns -PI/+PI, giving us an indication of direction to turn.
  // Between -PI/+PI also means we avoid an extreme demand sent to the heading PID.
  float diff = atan2( sin( ( demand_angle - RomiPose.theta) ), cos( (demand_angle - RomiPose.theta) ) );

  // If we have got the Romi theta to roughly match
  // the demand (by getting the difference to 0(ish)
  // We transition out of this behaviour.
  if ( abs( diff ) < 0.03 ) {

    // This ensures that the PID are reset
    // and sets the new STATE flag.
    changeState( STATE_INITIAL  );

  } else {    // else, turning behaviour

    // Measurement is the different in angle, demand is 0
    // bearing steers us to minimise difference toward 0
    float bearing = H_PID.update( 0, diff );

    // Append to motor speed control
    float l_pwr = L_PID.update( (0 - bearing), l_speed_t3 );
    float r_pwr = R_PID.update( (0 + bearing), r_speed_t3 );

    // Set motor power.
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);

  } // end of abs(diff)<0.03 if()

}// end of behaviour

void turnToThetaZero() {

  float demand_angle = 0;

  // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
  // Some crazy atan2 magic.
  // Treats the difference in angle as cartesian x,y components.
  // Cos and Sin are effectively wrapping the values between -1, +1, with a 90 degree phase.
  // So we can pass in values larger than TWO_PI or less than -TWO_PI fine.
  // atan2 returns -PI/+PI, giving us an indication of direction to turn.
  float diff = atan2( sin( ( demand_angle - RomiPose.theta) ), cos( (demand_angle - RomiPose.theta) ) );

  // If we have got the Romi theta to roughly match
  // the demand (by getting the difference to 0(ish)
  // We transition out of this behaviour.
  if ( abs( diff ) < 0.03 ) {
    changeState( STATE_INITIAL  );

  } else {    // else, turning behaviour

    // Measurement is the different in angle, demand is 0
    // bearing steers us to minimise difference toward 0
    float bearing = H_PID.update( 0, diff );

    // Append to motor speed control
    float l_pwr = L_PID.update( (0 - bearing), l_speed_t3 );
    float r_pwr = R_PID.update( (0 + bearing), r_speed_t3 );

    // Set motor power.
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);

  } // end of abs(diff)<0.03 if()

}// end of behaviour

void randomWalk() {

  // If we have been doing this for more than 6 seconds,
  // we transition out of this behaviour.
  unsigned long elapsed_t = (millis() - behaviour_t );
  if (  elapsed_t > 6000 ) {   // Change state.

    // This ensures that the PID are reset
    // and sets the new STATE flag.
    changeState( STATE_INITIAL );
    return;

  } else { // Otherwise, conduct a random walk behaviour

    // We change the motor demands at an interval of the
    // elapsed time using the remainder operation.
    if ( (elapsed_t % 10) == 0 ) { // if true, new turn.

      // Here, the turn is set to a gaussian number.
      // = randGaussian( mean, standard deviation)
      // So, it will draw a random number (+/-)
      // that average out to 0 over time, but have
      // a distribution S.D of +/- 5 (set below).
      // Therefore, small turns usually, sometimes big.
      float turn_bias = randGaussian( 0, 5 );
      float fwd_bias = 4;

      // Append to motor speed control
      float l_pwr = L_PID.update( (fwd_bias - turn_bias), l_speed_t3 );
      float r_pwr = R_PID.update( (fwd_bias + turn_bias), r_speed_t3 );

      L_Motor.setPower(l_pwr);
      R_Motor.setPower(r_pwr);

    } else {  // no update to turn, but update PID speed control

      // Here, we use a special method in our PID class which
      // updates the feedback assuming the demand has not changed.
      // The PID class stores the last_demand, so here we only
      // provide the measurement as an argument.
      float l_pwr = L_PID.update( l_speed_t3 );
      float r_pwr = R_PID.update( r_speed_t3 );

      // Set motor speeds.
      L_Motor.setPower(l_pwr);
      R_Motor.setPower(r_pwr);

    } // End of (elapsed_t % 10) if()
  }// End of (elapsed_t > 6000) if()

}// End of this behaviour.
