#ifndef _IRProximity_h
#define _IRProximity_h

class SharpIR {
    public:
        SharpIR(byte pin);
        int  getDistanceRaw();
        float  getDistanceInMM();
        void calibrate();

    private:
        byte pin;
};

SharpIR::SharpIR(byte _pin) {
  pin = _pin;
}

int SharpIR::getDistanceRaw( ) {
    return analogRead(pin);
}


/*
 * This piece of code is quite crucial to mapping
 * obstacle distance accurately, so you are encouraged
 * to calibrate your own sensor by following the labsheet.
 * Also remember to make sure your sensor is fixed to your
 * Romi firmly and has a clear line of sight!
 */
float SharpIR::getDistanceInMM() {
    
    float distance = (float)analogRead( pin );
    const float exponent = (1/-0.765);
    distance = pow( ( distance / 22432 ), exponent);
       
    return distance;
}


#endif
