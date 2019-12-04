// Pin definitions
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define LINE_LEFT_PIN   A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A2 //Pin for the right line sensor

#define IR_SENSOR A0

#define BUZZER 6

void setup() {

  pinMode( IR_SENSOR, INPUT );
  
  // Initialise the Serial communication
  Serial.begin( 9600 );
  delay(1000);
  Serial.println("***RESET***");
  
}


void loop() {

  int reading = analogRead( IR_SENSOR );
  Serial.println( reading );

  delay( 50 );
  
}
