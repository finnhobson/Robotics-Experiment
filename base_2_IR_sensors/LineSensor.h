#ifndef _LineSensor_h
#define _LineSensor_h
class LineSensor {
  public:
    LineSensor(int p0, int p1, int p2 ) {
      setPins( p0, p1, p2 );
      setGains( -0.01, 0.01 );
    };

    void setPins( int p0, int p1, int p2 ) {
      pin0 = p0;
      pin1 = p1;
      pin2 = p2;
      pinMode( p0, INPUT);
      pinMode( p1, INPUT);
      pinMode( p2, INPUT);
    }
    
    void calibrate() {
      bias0 = 0;
      bias1 = 0;
      bias2 = 0;
      for( int i = 0; i < 50; i++ ) {
         bias0 += analogRead( pin0 );
         bias1 += analogRead( pin1 );
         bias2 += analogRead( pin2 );
      }
      if( bias0 > 0 ) bias0 /= 50;
      if( bias1 > 0 ) bias1 /= 50;
      if( bias2 > 0 ) bias2 /= 50;
      
    }

    void setGains( float k0, float k1 ) {
        gain_left = k0;
        gain_right = k1;
    }

    boolean onLine( float threshold ) {
        
        // If any of these are true, return true.
        if( getCalibrated( LEFT )   > threshold ) return true;
        if( getCalibrated( CENTRE ) > threshold ) return true;
        if( getCalibrated( RIGHT )  > threshold ) return true;

       // Else, return false.
       return false;
        
    
    }

    void printRawSensorReadings() {
      Serial.print( analogRead( pin0 ) );
      Serial.print(",");
      Serial.print( analogRead( pin1 ) );
      Serial.print(",");
      Serial.print( analogRead( pin2 ) );
      Serial.print("\n");
    }

    void printCalibratedReadings() {

    }

    float getCalibrated( int which ) {
      float sensor_read;
      if( which == LEFT ) {
            sensor_read = (float)analogRead( pin0 );
            return (sensor_read - bias0 );
            
      } else if( which == CENTRE) {
            sensor_read = (float)analogRead( pin1 );
            return (sensor_read - bias1 );
            
      } else if( which == RIGHT ) {
            sensor_read = (float)analogRead( pin2 );
            return (sensor_read - bias2 );
            
      } else {  // non-existent "which" value
            return -1; 
      }
     }

    float getHeading( ) {

       float value_left   = getCalibrated( LEFT );
       float value_centre = getCalibrated( CENTRE );
       float value_right  = getCalibrated( RIGHT );
       float left, right;
       left = (value_left + value_centre );
       right = (value_centre + value_right);

       float feedback = (left * gain_left ) + (right * gain_right );
       
       return feedback;
    }


  private:
    int pin0;
    int pin1;
    int pin2;
    float bias0;
    float bias1;
    float bias2;
    float gain_left;
    float gain_right;
    const int LEFT    = 0;
    const int CENTRE  = 1;
    const int RIGHT   = 2;
};

#endif
