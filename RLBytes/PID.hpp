#ifndef RLPID_H
#define RLPID_H

int ZN_type = 0;
// Classic PID Benchmark =====================================================================
// Ref: Ziegler Nichols method PID
  float Ku = 1.65; // Critical or Ultimate kP which achieves neutal stability
  float Tu = 0.7; //ultimate cycle period (seconds)
  float Kp;
  float Ti;
  float Td;
  float Ki;
  float Kd;


  

  

struct StandardPID
{
    float E, P, I, D, prevE, prevI, limI, kP, kI, kD;
    unsigned long prevStamp;
    StandardPID(float limI_ = 1, float kP_ = 0, float kI_ = 0, float kD_ = 0);
    void reset();
    float update(float curPoint, float setPoint = 0); // all set prev and stiff
};

#endif // RLPID_H
