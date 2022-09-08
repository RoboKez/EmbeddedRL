#ifndef RLBYTES_H
#define RLBYTES_H

// Internals
#include "parameters.h"
#include "Agent.hpp"
#include "NN.hpp"
#include "env.hpp"
#include "EnvPri.hpp"
#include "EnvSec.hpp"


// Externals
#include "json.hpp"
#include "MemorySD.hpp"
using json = nlohmann::json;

//int seed = 3;
//bool rl = true;
//bool manual_reset = false;
//int n_epochs = 10; 
//int markov_time = 20;  // time between sending actions and getting next stage ms. I.e. 10 is 100Hz 

float episode_cum;
float batch_cum;
float batch_cum_det;
int eps;
int eps_det;
int total_training_steps = 0;
unsigned long iter_timestamp;
unsigned long soc_manual_time;
unsigned long soc_wait_time;
unsigned long det_manual_time;
unsigned long det_wait_time;
unsigned long train_time;
unsigned long test_time;
unsigned long learn_time;
unsigned long iter_time;

 

// PPO ==================================================================================================
Agent *agent = nullptr;
CoreEnv *env = nullptr;


void loadInfo(const char * folder, int iter);
void saveInfo();
void newExperiment(std::string env_name = "P_Primary",   
                    int step_limit = 600, 
                    int h1 = 8, 
                    int h2 = 8, 
                    float lr = 0.0003, 
                    int seed = seed, 
                    int batch_size = 600, 
                    int mini_batch_size = 64, 
                    bool adam = true,
                    std::string sub_env_folder = "",
                    int sub_env_iter = 0, 
                    int sub_repeats = 5);
                    
void loadExperiment(const char * folder, int iter);
void iterReset(); 
void ErrorMessageLoop(const char * message);


int iteration = -1;




#endif
