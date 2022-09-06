#include  "EnvSec.hpp"

// SecEnv ============================================================================================
SecEnv::SecEnv(std::string task_name, uint16_t ob_size, uint16_t act_size, uint16_t step_limit, std::string pri_policy_folder, uint16_t pri_iter, uint16_t sub_steps)
    : CoreEnv(task_name, ob_size, act_size, step_limit), m_sub_steps(sub_steps)
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
  
  m_sec_set_point = -100.5;
  // Obs .............................................................................................
   m_ob = secOb();
  m_ob[0] = secPID.P; //as different directions
  
  
 

  // Reward .........................................................................................
  m_reward = ((m_kill_angle/(m_kill_angle + abs(m_ob[0])))-0.5f) * 0.01;
  

  // Done  ..........................................................................................
  m_done = 0;

 
  // sub task says its in danger zone too long so done
  if (abs(m_pri_true) > m_kill_angle){
    m_steps_in_danger_zone += 1;
    
  }

  // current task says its in danger zone too long so done
  if (abs(m_ob[0]) > m_kill_speed){
    m_steps_in_danger_zone_sec += 1;
  }

  
  if (m_steps_in_danger_zone>m_steps_in_danger_zone_limit){
    m_done = 1; 
    m_reward =-0.1;
    setEyes(6);
    Serial.println("too much angle so died");
 

//  } else if (m_steps_in_danger_zone_sec>m_steps_in_danger_zone_limit_sec){
//    m_done = 1;
//     setEyes(7);
//     Serial.println("too much speed so died");
//

  
  } else if (m_step >= m_step_limit-1){
    m_done = 2;
     setEyes(8);
     Serial.println("step limit reached so died");


  }

   Serial.print("\nstep");
  Serial.print(m_step);
  Serial.print("\t ob");
  Serial.print(m_ob[0]);
  Serial.print("\t rew");
  Serial.println(m_reward,5);
  

  m_episode_return += m_reward;  // just for logging
}


// Step ----------------------------------------------------------------------------------------------
void SecEnv::markovStep(Eigen::VectorXf agent_act, int markovTime)
{

  m_pri_set_point = agent_act[0] ; 
  Serial.print("m_pri_set_point"); Serial.print('\t'); Serial.println(m_pri_set_point);

  for(uint16_t i=0; i<m_sub_steps; i++){
    VectorXf pri_ob = priOb();
    updateWheels(); // Also update vel to filter out some high frequency noise with the magnetic encoders
    priNet->m_in = priOb();
    priNet->ForwardPropagate();
    
    if(priNet->m_out.size() == 1){
      sendTorque(priNet->m_out[0], priNet->m_out[0]);
    } else if (priNet->m_out.size() == 2) {
      sendTorque(priNet->m_out[0], priNet->m_out[1]);
    } else {
      Serial.print("RL Error: agent act max is 2 actions, you have "); Serial.println(priNet->m_out.size());
    }
    delay(markovTime);
  }
  
  m_step +=1;
  getOb(); // Get next observation, rewards and done
}

// Reset ---------------------------------------------------------------------------------------------
unsigned long SecEnv::episodeReset(bool manual_reset)
{
  priPID.reset();
  m_episode_return = 0;
  sendTorque(0, 0);  // stop motors
  unsigned long manual_time = 0;

  // Have user center robot or start where last episode terminated
  if (manual_reset){
    unsigned long timestamp_reset = millis();
    m_pitch = 99.0f;
    while(m_pitch < -2.00 || m_pitch > 2.00){
      updatePitch();
      delay(10);
    }
    manual_time = timestamp_reset - millis();
    m_act[0] = 0;
    
  }
  setEyes(5);
  // Wait for sub task to orientate
    for(int i=0; i<30; i++){
      m_step = -1;
      Serial.print("resetting"); Serial.println(i);
      markovStep(m_act, 20);
    }
  setEyes(1);

  
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
