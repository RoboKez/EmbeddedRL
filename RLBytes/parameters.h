#ifndef PARAMETERS_H
#define PARAMETERS_H

float turn=0;

//uint8_t broadcastAddress[] = {0x7C, 0xDF, 0xA1, 0x10, 0x18, 0xE6};  //connect to dongle address to change set point

int control_mode          = 0 ;  // 0:PPO Learning, 1:non-learnt Zeigler-Nichols PID, 2:non-learnt Zeigler-Nichols PD, 3:non-learnt Zeigler-Nichols PI, 4:non-learnt Zeigler-Nichols P. 5: deployment joystick 

std::string primary_env   = "PID_Primary";
std::string sub_env       = "/PD_Sub";
int sub_iter              = 21;
int sub_repeats           = 5;
int step_limit            = 512;
int h1_size               = 8;
int h2_size               = 8;
float lr                  = 0.0003;
int seed                  = 3;
int batch_size            = 2048;  // must be greater or equal to minibatch size
int minibatch_size        = 64;
bool adam                 = true;
int n_epochs              = 10; 

int markov_time                   = 10;  // I.e. 10 is 100Hz (assume no latency which there is of 2-3ms so 100Hz is really 70Hz)
bool manual_reset                 = true;
float kill_angle                  = 0.9;
float steps_in_danger_zone_limit  = 1;
float standard_deviation          = 0.5;
float min_standard_deviation      = 0.3;
float anneal_rate                 = 0.0003;

bool smoothed_action              = false; // smooths transition from previous action
bool pposas                       = false; //make sure smoothed_action uis disabled

#endif // PARAMETERS_H
