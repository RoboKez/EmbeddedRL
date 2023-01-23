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
  
if(m_pri_ob_mode==5 ||m_pri_ob_mode==3){
  m_reward = ((1.0/(1.0 + abs(priPID.P)))) * 0.005;
  env->m_logrp += m_reward; //logging
  m_reward += ((1.0/(1.0 + abs(m_ob[2])))) * 0.005;
  env->m_logrc += ((1.0/(1.0 + abs(m_ob[2])))) * 0.005; //logging
} else {
  m_reward = ((1.0/(1.0 + abs(priPID.P)))) * 0.01;
  env->m_logrp += m_reward;
}

  // Done  ..........................................................................................
  m_done = 0;
  if (m_step >= m_step_limit-1){
    m_done = 2;
  }
  
  if (abs(m_pri_true) > m_kill_angle){
    m_steps_in_danger_zone += 1;
//    m_reward += -0.01;
  }
  if (m_steps_in_danger_zone>m_steps_in_danger_zone_limit){
    m_done = 1; 
  }

  if(m_pri_ob_mode==5 ||m_pri_ob_mode==3){
    if(abs(m_ob[2]) > 1.0){
      m_done = 1; 
    }
  }

  if(!incomingReadings.actL){ // user kill switch
     m_done = 2; 
     sendTorque(0, 0);
     setEyes(2);
     delay(8000);
     setEyes(3);
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
    updatePitch();

    while(m_pitch < -1.00 || m_pitch > 1.00){
      updatePitch();
      delay(markov_time);
    }
//    while(m_pitch < m_kill_angle*30.0f || m_pitch > 40.0f){
//      updatePitch();
//      delay(markov_time);
//    }
    updateWheels();
//    while(m_vL < -0.001f || m_vL > 0.001f){
//      updatePitch();
//      updateWheels();
//      delay(markov_time);
//    }
//    while(m_gyro < -1.0f || m_gyro > 1.0f){
//      updatePitch();
//      updateWheels();
//      delay(markov_time);
//    }
    
    manual_time = timestamp_reset - millis();
  }
//  m_ang_Start = 0.61

   priPID.reset();

   // A quick fix to save waiting for episode reset, assumes start conditions is its been stationary for a few seconds
    if(m_pitch < 0){
      priPID.I=priPID.limI;
    } else {
      priPID.I=-priPID.limI;
    }
   priPID.D = 0; 
   m_act[0] = 0;
   zeroWheelPos();
   
  for(int i=0; i<3; i++){
    markovStep(m_act,10);
  }
//  setEyes(3);

  m_step = -1;
  m_steps_in_danger_zone = 0;
  priPID.reset();
  getOb();
  return manual_time;
}
