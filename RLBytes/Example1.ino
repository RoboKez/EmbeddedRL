#include  "RLBytes.hpp"


void setup() 
{
  
  robotBegin();
//  newExperiment("P_Secondary", 
//                "/P_Primary_seed1", 
//                9,
//                5,
//                200,
//                8,
//                8,
//                0.0003,
//                1,
//                500,
//                64,
//                true);


  newExperiment("PI_Primary", 
                "", 
                0,
                0,
                200,
                8,
                8,
                0.0003,
                2, //seed
                500,
                64,
                true);


  
//  newExperiment("PD_Primary");
//  markov_time = 20;

//  loadExperiment("/Primary_Saved", 3);

}

void loop() 
{
  if(rl) // PPO  ===========================================================================================================================
  {
    iterReset(); // just used for logging
    // Test policy (deterministic) ---------------------------------------------------------------------------------------------------------
    env->m_pitch_inv = 1.0f;
    det_manual_time += env->episodeReset(manual_reset);
    while(env->m_done == 0 || env->m_done == 2)
    {
      agent->ForwardPropagate(env->m_ob);
      env->markovStep(agent->m_mu, markov_time);
    }
    float non_inv_return = env->getReturn();
    
    sendTorque(0, 0);
//    delay(5000);
    batch_cum_det += env->getReturn();

    eps_det +=1;
    
    test_time = iter_time - det_wait_time - det_manual_time;

    // Collect dataset (stochastic) --------------------------------------------------------------------------------------------------------
    unsigned long t1 = millis();
    setEyes(3);
    episode_cum = 0;
    while(agent->m_batch_id < agent->m_desired_batch_size){
      sendTorque(0, 0);
//      delay(5000);
      soc_manual_time += env->episodeReset(manual_reset);
      while(!env->m_done)
      {
        agent->ForwardPropagate(env->m_ob);
        env->markovStep(agent->m_act, markov_time);
        agent->CacheSample(env->m_reward, env->m_done, env->m_ob);
        total_training_steps += 1;
        episode_cum += env->m_reward;  // logging
      }
      sendTorque(0, 0);
      batch_cum += env->getReturn();
//      soc_wait_time += 500;
      eps += 1;
    }
    train_time = t1 - soc_wait_time - soc_manual_time;
      
    // Learn --------------------------------------------------------------------------------------------------------------------------------
    unsigned long t2 = millis();
    setEyes(4);
    sendTorque(0, 0);
    saveInfo();
    agent->SaveNetworkSD(env->m_folder, iteration); 
    agent->UpdatePPO(n_epochs);
    learn_time = t2 - millis();
    


  } else {  // Hardcoded Ziegler Nichols PID ===============================================================================================
    ZN_type = 0;
    markov_time = 10;
    if(ZN_type==0){ // PID controller
       Kp = Ku * 0.6f;
       Ti = Tu / 2.0f;
       Td = Tu / 8.0f;
      } else if(ZN_type==1){ // PI controller
       Kp = Ku * 0.45f;
       Ti = Tu / 1.2f;
       Td = 0.0f;
      } 
      Ki = Kp/Ti;
      Kd = Kp*Td;
    StandardPID standPID(0.2, Kp, Ki, Kd);

    standPID.reset();
    env->zeroWheelPos();
    while(1){
      Serial.println(env->m_ob[0]);
      agent->m_act[0] =  standPID.update(env->m_pri_true, 0);
      env->markovStep(agent->m_act, markov_time);
    }
  }
}
