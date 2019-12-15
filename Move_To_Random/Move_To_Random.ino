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

#define STRAIGHT_PWM          40.0
#define TURN_PWM              21.0

// Behaviour parameters
#define LINE_THRESHOLD        200.00
#define STRAIGHT_FWD_SPEED    10.0
#define LINE_FOLLOW_SPEED     8.0
#define IR_DETECTD_THRESHOLD  85   // a close reading in mm (danger)
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
int desiredX;
int desiredY;
bool left_turn_last = true; //turn right first
float last_line_x;
float last_line_y;

// used by timer3.h to calculate left and right wheel speed.
volatile float l_speed_t3, r_speed_t3;

// Different states(behaviours) the robot
// can be in.
int STATE;
#define STATE_CALIBRATE       0    // calibrates line sensor
#define STATE_DRIVE_STRAIGHT  1    // Robot drives in a straight line until map edge or obstacle
#define STATE_AVOID_OBJECT    2    // Robot turns to random angle at object
#define STATE_NEW_DEST        3




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
  RomiPose.setPose( 1, 1, 0 );

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

    }

    else {
      Map.updateMapFeature( '.' , RomiPose.x, RomiPose.y );
    }

    //    //Check for obstacles
    //    //if ( SERIAL_ACTIVE ) Serial.println( IRSensor0.getDistanceInMM() );
    //    if ( IRSensor0.getDistanceInMM() < IR_DETECTD_THRESHOLD ) {
    //
    //      // Record that we found an obstruction on the LEFT
    //      // Using LED just to see if it is working.
    //      digitalWrite(DEBUG_LED, HIGH);
    //
    //      obstacleUpdateLeft();
    //
    //      // Set next state to obstacle avoidance,
    //      // caught be switch below.
    //      changeState( STATE_NEW_DEST );
    //    }
    //
    //    if ( IRSensor1.getDistanceInMM() < IR_DETECTD_THRESHOLD ) {
    //
    //      // Record that we found an obstruction on the RIGHT
    //      // Using LED just to see if it is working.
    //      digitalWrite(DEBUG_LED, HIGH);
    //
    //      obstacleUpdateRight();
    //
    //      // Set next state to obstacle avoidance,
    //      // caught be switch below.
    //      changeState( STATE_NEW_DEST );
    //
    //    } else {
    //      digitalWrite(DEBUG_LED, LOW);
    //    }


    // Choose relevant helper functioned based on current state
    switch ( STATE ) {

      case STATE_CALIBRATE:
        calibrateSensors();
        break;

      case STATE_DRIVE_STRAIGHT:
        driveStraight();
        break;

      case STATE_AVOID_OBJECT:
        avoidObject();
        break;

      case STATE_NEW_DEST:
        turnToDemand();
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
  if (  elapsed_t > 10 ) {
    behaviour_t = millis();
    Serial.print(RomiPose.x);
    Serial.print(" , ");
    Serial.print(RomiPose.y);
    Serial.print(" , ");
    Serial.println(RomiPose.theta);

  }

  if ( RomiPose.x <= desiredX + 10 && RomiPose.x >= desiredX - 10 && RomiPose.y <= desiredY + 10 && RomiPose.y >= desiredY - 10 ) {
    changeState( STATE_NEW_DEST );
  }

  float theta_error = demand_angle - RomiPose.theta;
  int turn_pwm = 0;

  if (theta_error > 0) {
    turn_pwm = -3;
  }
  else if (theta_error < 0) {
    turn_pwm = 3;
  }
  else turn_pwm = 0;

  int left_demand = STRAIGHT_PWM - turn_pwm;
  int right_demand = STRAIGHT_PWM + 2 + turn_pwm;

  L_Motor.setPower(left_demand);
  R_Motor.setPower(right_demand);

}


void avoidObject() {

  unsigned long elapsed_t = (millis() - behaviour_t );
  if (  elapsed_t > turn_time ) {
    changeState( STATE_NEW_DEST );
  }

  else {
    L_Motor.setPower((float) - TURN_PWM);
    R_Motor.setPower((float)TURN_PWM + 2);
  }
}

void getLocation() {
  int loc = Map.returnEmptySquare();
  desiredY = loc % MAP_RESOLUTION;
  desiredX = (loc - desiredY) / MAP_RESOLUTION;
  desiredY = Map.indexToPose(desiredY, MAP_Y, MAP_RESOLUTION);
  desiredX = Map.indexToPose(desiredX, MAP_X, MAP_RESOLUTION);
  Serial.print(desiredX);
  Serial.print(" , ");
  Serial.println(desiredY);
}

void turnToDemand() {
  float diff = abs(RomiPose.theta - demand_angle);
  if (  diff < 0.001 ) {
    changeState( STATE_DRIVE_STRAIGHT );
  }
  else {
    if (RomiPose.theta - demand_angle < 0) {
      L_Motor.setPower((float)TURN_PWM);
      R_Motor.setPower((float) - TURN_PWM - 2);
    }
    else {
      L_Motor.setPower((float) - TURN_PWM);
      R_Motor.setPower((float)TURN_PWM + 2);
    }
  }
}

float angleOfLoc() {
  float angleOfLoc = 0;
  int xDiff   =  desiredX - RomiPose.x;
  int yDiff   =  desiredY - RomiPose.y;
  angleOfLoc = 0;

  if (xDiff == 0) {
    if (yDiff >= 0) {
      angleOfLoc = PI / 2;
    }
    else {
      angleOfLoc = -PI / 2;
    }
  }
  else if (xDiff > 0) {
    if (yDiff >= 0) {
      angleOfLoc =    atan2(yDiff, xDiff);
    }
    else {
      angleOfLoc =  - atan2(-yDiff, xDiff);
    }
  }
  else {
    if (yDiff >= 0) {
      angleOfLoc =  - atan2(yDiff, -xDiff) + PI;
    }
    else {
      angleOfLoc =    atan2(-yDiff, -xDiff) + PI;
    }
  }

  while ( angleOfLoc < -PI ) angleOfLoc += TWO_PI;
  while ( angleOfLoc > PI ) angleOfLoc -= TWO_PI;
  Serial.println(angleOfLoc);

  return angleOfLoc;


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

  if (which == STATE_DRIVE_STRAIGHT) {
    demand_angle = RomiPose.theta;
  }
  if (which == STATE_AVOID_OBJECT) {
    turn_time = random(1500, 3500);
  }
  if (which == STATE_NEW_DEST) {
    getLocation();
    demand_angle = angleOfLoc();
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
  return;
}


void calibrateSensors() {

  // Make sure motors are off so the robot
  // stays still.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );


  // Line sensor.
  LineSensor.calibrate();

  // After calibrating, we send the robot to initial state.
  changeState( STATE_NEW_DEST );
}

void obstacleUpdateLeft() {
  float angle_romi = RomiPose.theta;
  float angle_sensor = angle_romi - 0.0349;
  if ( angle_sensor < 0 ) {
    angle_sensor = 2 * PI + angle_sensor;
  }
  float total_dist = IRSensor0.getDistanceInMM();
  total_dist =  total_dist + 143 / 2;
  if ( angle_sensor > 0 && angle_sensor < PI / 2 ) {
    float dist_x = cos(angle_sensor) * total_dist;
    float dist_y = sin(angle_sensor) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x + dist_x, RomiPose.y + dist_y );
  }
  else if ( angle_sensor > PI / 2 && angle_sensor < PI ) {
    float new_angle = PI - angle_sensor;
    float dist_x = cos(new_angle) * total_dist;
    float dist_y = sin(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x - dist_x, RomiPose.y + dist_y );
  }
  else if (angle_sensor > PI && angle_sensor < 3 * PI / 2 ) {
    float new_angle = angle_sensor - PI;
    float dist_x = cos(new_angle) * total_dist;
    float dist_y = sin(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x - dist_x, RomiPose.y - dist_y );
  }
  else if (angle_sensor > 3 * PI / 2 && angle_sensor < 2 * PI ) {
    float new_angle = angle_sensor - 3 * PI / 2;
    float dist_x = sin(new_angle) * total_dist;
    float dist_y = cos(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x + dist_x, RomiPose.y - dist_y );
  }
}// End of this behaviour.

void obstacleUpdateRight() {
  float angle_romi = RomiPose.theta * (360 / TWO_PI);
  float angle_sensor = angle_romi + 0.0349;
  if ( angle_sensor > 2 * PI ) {
    angle_sensor = angle_sensor - 2 * PI;
  }
  float total_dist = IRSensor0.getDistanceInMM();
  total_dist =  total_dist + 143 / 2;
  if ( angle_sensor > 0 && angle_sensor < PI / 2 ) {
    float dist_x = cos(angle_sensor) * total_dist;
    float dist_y = sin(angle_sensor) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x + dist_x, RomiPose.y + dist_y );
  }
  else if ( angle_sensor > PI / 2 && angle_sensor < PI ) {
    float new_angle = PI - angle_sensor;
    float dist_x = cos(new_angle) * total_dist;
    float dist_y = sin(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x - dist_x, RomiPose.y + dist_y );
  }
  else if (angle_sensor > PI && angle_sensor < 3 * PI / 2 ) {
    float new_angle = angle_sensor - PI;
    float dist_x = cos(new_angle) * total_dist;
    float dist_y = sin(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x - dist_x, RomiPose.y - dist_y );
  }
  else if (angle_sensor > 3 * PI / 2 && angle_sensor < 2 * PI ) {
    float new_angle = angle_sensor - 3 * PI / 2;
    float dist_x = sin(new_angle) * total_dist;
    float dist_y = cos(new_angle) * total_dist;
    Map.updateMapFeature( 'O' , RomiPose.x + dist_x, RomiPose.y - dist_y );
  }
}// End of this behaviour.
