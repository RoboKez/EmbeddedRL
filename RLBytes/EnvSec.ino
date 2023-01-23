#include  "EnvSec.hpp"

// SecEnv ============================================================================================
SecEnv::SecEnv(std::string task_name, uint16_t ob_size, uint16_t act_size, uint16_t step_limit, std::string pri_policy_folder, uint16_t pri_iter, uint16_t sub_steps)
    : CoreEnv(task_name, ob_size, act_size, step_limit, pri_policy_folder), m_sub_steps(sub_steps)
{
  // Load subpolicy
  m_steps_in_danger_zone_limit = 1;
  
  Serial.print(task_name.c_str()); Serial.println("Env constructed");
  LoadSubPolicy(pri_policy_folder, pri_iter);
}

SecEnv::~SecEnv()
{
  Serial.println("SecEnv destroyed");
}

// Get Ob, Reward, Done ------------------------------------------------------------------------------
void SecEnv::getOb()
{
  
  m_sec_set_point = -3.14;
  // Obs .............................................................................................
   m_ob = secOb();
  m_ob[0] = secPID.P; //as different directions
 
  // Reward .........................................................................................
//  m_reward = ((m_kill_angle/(m_kill_angle + abs(m_ob[0])))-0.5f) * 0.01; 
  m_reward = prev_pos - m_ob[0]; 
//  Serial.println(m_reward);

  // Done  ..........................................................................................
  m_done = 0;
  if (abs(m_pri_true) > m_kill_angle){
     m_done = 1;
  } else if(abs(m_pL) > 6.28){
    m_done = 1; 
  } else if (m_step >= m_step_limit-1){
    m_done = 2;
  }
  m_episode_return += m_reward;  // just for logging
  prev_pos = m_ob[0];
}


// Step ----------------------------------------------------------------------------------------------
void SecEnv::markovStep(Eigen::VectorXf agent_act, int markovTime)
{

  m_pri_set_point = agent_act[0]*0.2; 
//  Serial.print("m_pri_set_point"); Serial.print('\t'); Serial.println(m_pri_set_point);

  for(uint16_t i=0; i<sub_repeats; i++){
//    VectorXf pri_ob = priOb();
    priNet->m_in = priOb();
    priNet->ForwardPropagate();
    sendTorque(priNet->m_out[0], priNet->m_out[0]);
    delay(markovTime);
  }
  
  m_step +=1;
  getOb(); // Get next observation, rewards and done
}

// Reset ---------------------------------------------------------------------------------------------
unsigned long SecEnv::episodeReset(bool manual_reset)
{ 
  sendTorque(0,0); 
  delay(2000);
  priPID.reset();
  m_episode_return = 0;
  sendTorque(0, 0);  // stop motors
  unsigned long manual_time = 0;

//  setEyes(5);
  m_act[0] = 0; // this is pitch set point
  // Wait for sub task to orientate
    for(int i=0; i<40; i++){
      m_step = -1;
//      Serial.print("resetting"); Serial.println(i);
      markovStep(m_act, markov_time);
    }
//  setEyes(1);

  zeroWheelPos();
  m_step = -1;
  m_steps_in_danger_zone = 0;
  m_steps_in_danger_zone_sec = 0;
  priPID.reset();
  secPID.reset();
  getOb();
  return manual_time;
}

// Load sub policy --------------------------------------------------------------------------------------
void SecEnv::LoadSubPolicy(std::string folder, int iter){

  char file[100] = "/policy_";
  char str_iter[5];
  sprintf(str_iter, "%d", iter);
  char xPath[100];
  strcpy(xPath, folder.c_str());
  strcat(xPath, file);
  strcat(xPath, str_iter);
  strcat(xPath, ".json");
  Serial.print("load sub policy info at "); Serial.println(xPath);
  nlohmann::json j = readJsonFile(SD, xPath);
  delete priNet;
  priNet = new NeuralNetwork(j[0]["W1"][0].size(),
                             j[0]["W3"].size(),
                             j[0]["W1"].size(),
                             j[0]["W2"].size()
                             );
  char pFile[100] = "/policy_";        
  priNet->LoadNetworkSD(folder.c_str(), iter, pFile);



  priNet->PrintWeights();
}
