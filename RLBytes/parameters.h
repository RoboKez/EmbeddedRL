#ifndef PARAMETERS_H
#define PARAMETERS_H

int control_mode          = 0;  // 0:PPO Learning, 1:Zeigler-Nichols PID, 2:Zeigler-Nichols PD, 3:Zeigler-Nichols PI, 4:Zeigler-Nichols P  
std::string primary_env   = "Classic_Primary";
int step_limit            = 200;
int h1_size               = 8;
int h2_size               = 8;
float lr                  = 0.0003;
int seed                  = 1;
int batch_size            = 600;
int minibatch_size        = 64;
bool adam                 = true;
int n_epochs              = 3; 

int markov_time           = 20;  // I.e. 10 is 100Hz 
bool smoothed_action      = false; // smooths transition from previous action
bool manual_reset         = true;

float kill_angle                  = 0.65;
float steps_in_danger_zone_limit  = 1;

float standard_deviation          = 0.5;
float min_standard_deviation      = 0.3;
float anneal_rate                 = 0.0003;


#endif // PARAMETERS_H
