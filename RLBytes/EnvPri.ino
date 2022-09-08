#include  "EnvPri.hpp"

// PriEnv ============================================================================================
PriEnv::PriEnv(std::string task_name, uint16_t ob_size, uint16_t act_size, uint16_t step_limit)
    : CoreEnv(task_name, ob_size, act_size, step_limit)
{
  Serial.print("pri");
  Serial.print(task_name.c_str()); Serial.println("Env constructed");
}

PriEnv::~PriEnv()
{
  Serial.println("PriEnv destroyed");
}

// Get Ob, Reward, Done ------------------------------------------------------------------------------
void PriEnv::getOb()
{
  // Obs .............................................................................................
  m_ob = priOb();  // Update ob depending on task (see CoreEnv)

  // Reward .........................................................................................
  m_reward = ((1.0/(1.0 + abs(priPID.P)))) * 0.01;
  
//m_reward = ((m_kill_angle/(m_kill_angle + abs(priPID.P)))) * 0.01;

if(m_pri_ob_mode==5){
  m_reward += ((m_kill_angle/(m_kill_angle + abs(m_ob[2])))) * 0.01;
}

//  m_reward  = 0.01;


  // Done  ..........................................................................................
  m_done = 0;
  if (m_step >= m_step_limit-1){
    m_done = 2;
  }
  
  if (abs(m_pri_true) > m_kill_angle){
    m_steps_in_danger_zone += 1;
    m_reward = -0.01;
  }
  if (m_steps_in_danger_zone>m_steps_in_danger_zone_limit){
    m_done = 1; 
  }
  

  m_episode_return += m_reward;  // just for logging
}


// Reset ---------------------------------------------------------------------------------------------
unsigned long PriEnv::episodeReset(bool manual_reset)
{
  m_episode_return = 0;
  sendTorque(0, 0);  // stop motors
  unsigned long manual_time = 0;

  // Have user center robot or start where last episode terminated
  if (manual_reset){
    unsigned long timestamp_reset = millis();
    m_pitch = 99.0f;
    while(m_pitch < -2.00 || m_pitch > 2.00){
      updatePitch();
      delay(markov_time);
    }
    manual_time = timestamp_reset - millis();
  }

   priPID.reset();

   // A quick fix to save waiting for episode reset, assumes start conditions is its been stationary for a few seconds
    if(m_pitch < 0){
      priPID.I=priPID.limI;
    } else {
      priPID.I=-priPID.limI;
    } 
   m_act[0] = 0;
   zeroWheelPos();
   
  for(int i=0; i<3; i++){
    markovStep(m_act,20);
  }

  m_step = -1;
  m_steps_in_danger_zone = 0;
  priPID.reset();
  getOb();
  return manual_time;
}
