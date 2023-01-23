#include "pid.hpp"

float StandardPID::update(float curPoint, float setPoint){
    unsigned long curStamp = micros();
    float timestep = float(curStamp - prevStamp) * 1e-6f;
    if(timestep <= 0 || timestep > 0.5f) timestep = 1e-3f;  // Ref: Fix taken from SimpleFOC

    E = setPoint - curPoint;
    P = E;
    I = (E + prevE) * timestep * 0.5f + prevI;
    D = (E - prevE) / timestep;
//    if(D < -10.0f || D > 10.0f) D = 0;  // Fix for episode reset
//    D = constrain(D, -2.0, 2.0);
//    P = constrain(P, -2.0, 2.0);
//    I = constrain(I, -1.0, 1.0);

    float output = kP*P + kI*I + kD*D;
    
    prevStamp = curStamp;
    prevE = E;
    prevI = I;

    output = constrain(output, -1.0, 1.0);

    return output;
}


// StandardPID ===========================================================================================

StandardPID::StandardPID(float limI_, float kP_, float kI_, float kD_)
:kP(kP_), kI(kI_), kD(kD_), limI(limI_){
}

// Reset -------------------------------------------------------------------------------------------------
void StandardPID::reset(){
    E = 0;
    P = 0;
    I = 0;
    D = 0;
    prevE = 0;
    prevI = 0;
    prevStamp = micros();
    Serial.println("PID reset");
}
