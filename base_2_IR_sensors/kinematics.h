#ifndef _Kinematics
#define _Kinematics_h


class Kinematics
{
  public:
    //Public variables and methods go here
    float x;
    float y; 
    float theta;
    float last_theta;
    long last_left;
    long last_right;

    Kinematics::Kinematics();
    void update( long left, long right );
    void setPose( float _x, float _y, float _theta );

  private:
    
    float wheel_sep         = (143 / 2);
    float wheel_radius      = 35;
    float rad_per_enc_step  = (TWO_PI / 1440.0);

}; // End of class definition.


Kinematics::Kinematics() {
  x = 0;
  y = 0;
  theta = 0;
  last_theta = 0;
  last_left = 0;
  last_right = 0;
} // end of constructor.


void Kinematics::setPose( float _x, float _y, float _theta ) {
  x     = _x;
  y     = _y;
  theta = _theta;
  
}

void Kinematics::update( long left, long right ) {

  // This class stores the last received
  // encoder counts to calculate the difference
  // in encoder counts for itself.
  long delta_left = left - last_left;
  long delta_right = right - last_right;
  last_left = left;
  last_right = right;

  // Rotation for each wheel, using
  // the number of radians per encoder count
  // We're only keeping track of distance travelled
  // (a cartesian coordinate position), not speed, 
  // so we don't need to factor elapsed time here.
  // However, knowing how long a time elapsed between
  // update() might help to estimate error in the 
  // approximation (?)
  float v0;
  v0 = rad_per_enc_step;
  v0 = v0 * (float)delta_left;

  float v1;
  v1 = rad_per_enc_step;
  v1 = v1 * (float)delta_right;


  // Kinematics without ICC projection
  // Some error is going to accumulate.
  // But with a quick enough update, its a pretty good
  // straight-line approximation for curves.
  float new_x = (( v0 * wheel_radius ) + ( v1 * wheel_radius )) / 2;
  
  float new_theta = (( v0 * wheel_radius) - (v1 * wheel_radius ) ) / (2 * wheel_sep );

  // record current theta as 'old' so that we can
  // keep track of angular change else where in 
  // the program.
  last_theta = theta;
  

  // Set new theta.
  theta = theta + new_theta;

  // Lets wrap theta to keep it between 0 and TWO_PI
  // Not strictly necessary, but predictable at least.
  while( theta < -PI ) theta += TWO_PI;
  while( theta > PI ) theta -= TWO_PI;

  // Integrate this movement step by rotating
  // the x contribution by theta, therefore "sharing" it
  // between x and y in the global reference frame.
  x = x + (new_x * cos( theta ) );
  y = y + (new_x * sin( theta ) );

} // End of update()





#endif
