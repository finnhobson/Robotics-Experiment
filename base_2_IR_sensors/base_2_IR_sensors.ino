
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

#define IR_PROX_PIN_L    A0   //IR Sensors
#define IR_PROX_PIN_R    A1

#define DEBUG_LED      13   // Using the orange LED for debugging

#define BUTTON_A       14   // Push button labelled A on board.
#define BUTTON_B       30   // Push button labelled B on board.

// Behaviour parameters
#define LINE_THRESHOLD        200.00
#define STRAIGHT_FWD_SPEED    10.0
#define LINE_FOLLOW_SPEED     8.0
#define IR_DETECTD_THRESHOLD  100   // a close reading in mm (danger)
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
SharpIR     IRSensor0( IR_PROX_PIN_L );                      //Get distance to objects (incomplete class)
SharpIR     IRSensor1( IR_PROX_PIN_R );
Kinematics  RomiPose;                                       // Not using ICC.
Mapper      Map;                                            // Default: 25x25 grid, 72mm resolution.


/*****************************************************************************
    OTHER GLOBAL VARIABLES
*****************************************************************************/


unsigned long update_t;   // Used for timing/flow control for main loop()
unsigned long behaviour_t;// Use to track how long a behaviour has run.

bool angle_setup;
float demand_angle;

float turn_time;
float turnPWM = 25.0f;
int drivePWM = 30;

float last_line_x;
float last_line_y;

// used by timer3.h to calculate left and right wheel speed.
volatile float l_speed_t3, r_speed_t3;

// Different states(behaviours) the robot
// can be in.
int STATE;
#define STATE_CALIBRATE       0    // calibrates line sensor
#define STATE_DRIVE_STRAIGHT  1    // Robot drives in a straight line until map edge or obstacle
#define STATE_AVOID_EDGE      2    // Robot turns to random angle at edge
#define STATE_AVOID_OBJECT    3    // Robot turns to random angle at object




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
  RomiPose.setPose( 5, MAP_Y / 2, 0 );

  Map.start();

  angle_setup = false;

  demand_angle = 0;

  randomSeed(analogRead(0));

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
  RomiPose.update( left_count, right_count );
  

  // Runs a behaviour every 50ms, skips otherwise.
  // Therefore, behaviours update 20 times a second
  if (  millis() - update_t > 5 ) {
    update_t = millis();

    /*Serial.print( STATE );
      Serial.print( ", " );
      Serial.print( demand_angle );
      Serial.print( ", " );
      Serial.println( RomiPose.theta );*/

    // We check for a line, and if we find one
    // we immediately change state to line following.
    if ( LineSensor.onLine( LINE_THRESHOLD ) ) {

      // Record that we found a line.
      Map.updateMapFeature( 'L' , RomiPose.x, RomiPose.y );
      last_line_x = RomiPose.x;
      last_line_y = RomiPose.y;

      // Set next state to line following, caught
      // by switch below
      //changeState( STATE_FOLLOW_LINE );

    }

    else {
      Map.updateMapFeature( '.' , RomiPose.x, RomiPose.y );
    }

    //Check for obstacles
    //if ( SERIAL_ACTIVE ) Serial.println( IRSensor0.getDistanceInMM() );
    if ( IRSensor0.getDistanceInMM() < IR_DETECTD_THRESHOLD ) {

      // Record that we found an obstruction on the LEFT
      // Using LED just to see if it is working.
      digitalWrite(DEBUG_LED, HIGH);

      obstacleUpdateLeft();

      // Set next state to obstacle avoidance,
      // caught be switch below.
      changeState( STATE_AVOID_OBJECT );
    }

    if ( IRSensor1.getDistanceInMM() < IR_DETECTD_THRESHOLD ) {

      // Record that we found an obstruction on the RIGHT
      // Using LED just to see if it is working.
      digitalWrite(DEBUG_LED, HIGH);

      obstacleUpdateRight();

      // Set next state to obstacle avoidance,
      // caught be switch below.
      changeState( STATE_AVOID_OBJECT );

    } else {
      digitalWrite(DEBUG_LED, LOW);
    }


    // Choose relevant helper functioned based on current state
    switch ( STATE ) {

      case STATE_CALIBRATE:
        calibrateSensors();
        break;

      case STATE_DRIVE_STRAIGHT:
        driveStraight();
        break;

      case STATE_AVOID_EDGE:
        avoidEdge();
        break;

      case STATE_AVOID_OBJECT:
        avoidObject();
        break;

      default: // unknown, this would be an error.
        reportUnknownState();
        break;

    } // End of state machine switch()

  } // End of millis

  // Small delay to prevent millis = 0
  delay(1);

}// End of Loop()



void driveStraight() {

  // If we reach the edge of the map, switch to changeAngle behaviour
  unsigned long elapsed_t = (millis() - behaviour_t );
  if (  elapsed_t > 500 ) {
    if ( RomiPose.x <= 1 || RomiPose.y <= 1 || RomiPose.x >= (MAP_X - 1) || RomiPose.y >= (MAP_Y - 1) ) {
      changeState( STATE_AVOID_EDGE );
    }
  }

  float theta_error = demand_angle - RomiPose.theta;
  int turn_pwm = 0;

  if (theta_error > 0) {
    turn_pwm = -2;
  }
  else if (theta_error < 0) {
    turn_pwm = 2;
  }
  else turn_pwm = 0;

  int left_demand = drivePWM - turn_pwm;
  int right_demand = drivePWM + turn_pwm;

  L_Motor.setPower(left_demand);
  R_Motor.setPower(right_demand);

}


void avoidObject() {

  unsigned long elapsed_t = (millis() - behaviour_t );
  if (  elapsed_t > turn_time ) {
    changeState( STATE_DRIVE_STRAIGHT );
  }

  else {
    L_Motor.setPower(-turnPWM);
    R_Motor.setPower(turnPWM);
  }
}


void avoidEdge() {

  float diff = abs(RomiPose.theta - demand_angle);
  if (  diff < 0.02 ) {
    changeState( STATE_DRIVE_STRAIGHT );
  }

  else {
    L_Motor.setPower(-turnPWM);
    R_Motor.setPower(turnPWM);
  }
}

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

  // A short beep if debugging
  beep();

  if (which == 1) {
    demand_angle = RomiPose.theta;
  }
  if (which == 2) {
    demand_angle = pickAngle();
  }
  if (which == 3) {
    turn_time = random(1500, 3500);
  }

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


float pickAngle() {
  float angle = 0.0f;

  bool xBottom = RomiPose.x <= 30;
  bool xTop = RomiPose.x >= (MAP_X - 30);
  bool yBottom = RomiPose.y <= 30;
  bool yTop = RomiPose.y >= (MAP_Y - 30);

  if (xBottom && yBottom) {
    angle = random(0, (PI / 2) * 100) / 100.0f;
    Serial.println("BOTTOM LEFT");
  }
  else if (xBottom && yTop) {
    angle = random((-PI / 2) * 100, 0) / 100.0f;
    Serial.println("BOTTOM RIGHT");
  }
  else if (xTop && yBottom) {
    angle = random((PI / 2) * 100, PI * 99) / 100.0f;
    Serial.println("TOP LEFT");
  }
  else if (xTop && yTop) {
    angle = random((-PI) * 99, (-PI / 2) * 100) / 100.0f;
    Serial.println("TOP RIGHT");
  }
  else if (xBottom) {
    angle = random((-PI / 2) * 100, (PI / 2) * 100) / 100.0f;
    Serial.println("BOTTOM");
  }
  else if (yBottom) {
    angle = random(0, PI * 99) / 100.0f;
    Serial.println("LEFT");
  }
  else if (xTop) {
    angle = random((PI / 2) * 100, (3 * (PI / 2)) * 100) / 100.0f;
    Serial.println("TOP");
  }
  else if (yTop) {
    angle = random((-PI) * 99, 0) / 100.0f;
    Serial.println("RIGHT");
  }

  while ( angle < -PI ) angle += TWO_PI;
  while ( angle > PI ) angle -= TWO_PI;
  Serial.println(angle);
  return angle;
}


void calibrateSensors() {

  // Make sure motors are off so the robot
  // stays still.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );


  // Line sensor.
  LineSensor.calibrate();

  // After calibrating, we send the robot to initial state.
  changeState( STATE_DRIVE_STRAIGHT );
}

void obstacleUpdateLeft() {
  float angle_romi = RomiPose.theta * (360 / TWO_PI);
  float angle_sensor = angle_romi - 20;
  if ( angle_sensor < 0 ) {
    angle_sensor = 360 + angle_sensor;
  }
  float total_dist = IRSensor0.getDistanceInMM();
  if ( angle_sensor > 0 && angle_sensor < 90 ) {
    float dist_x = sin(angle_sensor) * total_dist;
    float dist_y = cos(angle_sensor) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x + dist_x, RomiPose.y + dist_y );
  }
  else if ( angle_sensor > 90 && angle_sensor < 180 ) {
    float new_angle = 180 - angle_sensor;
    float dist_x = cos(new_angle) * total_dist;
    float dist_y = sin(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x - dist_x, RomiPose.y + dist_y );
  }
  else if (angle_sensor > 180 && angle_sensor < 270 ) {
    float new_angle = angle_sensor - 180;
    float dist_x = cos(new_angle) * total_dist;
    float dist_y = sin(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x - dist_x, RomiPose.y - dist_y );
  }
  else if (angle_sensor > 270 && angle_sensor < 360 ) {
    float new_angle = angle_sensor - 270;
    float dist_x = sin(new_angle) * total_dist;
    float dist_y = cos(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x + dist_x, RomiPose.y - dist_y );
  }
}// End of this behaviour.

void obstacleUpdateRight() {
  float angle_romi = RomiPose.theta * (360 / TWO_PI);
  float angle_sensor = angle_romi + 20;
  if ( angle_sensor > 360 ) {
    angle_sensor = angle_sensor - 360;
  }
  float total_dist = IRSensor0.getDistanceInMM();
  if ( angle_sensor > 0 && angle_sensor < 90 ) {
    float dist_x = sin(angle_sensor) * total_dist;
    float dist_y = cos(angle_sensor) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x + dist_x, RomiPose.y + dist_y );
  }
  else if ( angle_sensor > 90 && angle_sensor < 180 ) {
    float new_angle = 180 - angle_sensor;
    float dist_x = cos(new_angle) * total_dist;
    float dist_y = sin(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x - dist_x, RomiPose.y + dist_y );
  }
  else if (angle_sensor > 180 && angle_sensor < 270 ) {
    float new_angle = angle_sensor - 180;
    float dist_x = cos(new_angle) * total_dist;
    float dist_y = sin(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x - dist_x, RomiPose.y - dist_y );
  }
  else if (angle_sensor > 270 && angle_sensor < 360 ) {
    float new_angle = angle_sensor - 270;
    float dist_x = sin(new_angle) * total_dist;
    float dist_y = cos(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x + dist_x, RomiPose.y - dist_y );
  }
}// End of this behaviour.
