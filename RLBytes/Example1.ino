#include  "RLBytes.hpp"
//float setByte = 127;

void setup() 
{
  robotBegin();

//  newExperiment(primary_env, 
//                step_limit,
//                h1_size,
//                h2_size,
//                lr,
//                seed,
//                batch_size,
//                minibatch_size,
//                adam,
//                sub_env,
//                sub_iter);

loadExperiment("/PI_Primary_1", 10);

//loadExperiment("/Classic_Primarywild1", 24);

//loadExperiment("/Classic_Primaryvidman5", 0);
//  loadExperiment("/P_Secondary", 3, "/PD_Sub", 20);
//delay(5000); //allow the user time to roughly orientate
//Serial.print("here");
}
VectorXf pitch_log = VectorXf(3000);
VectorXf act_log = VectorXf(3000);
VectorXf pitch_D_log = VectorXf(3000);
VectorXf pitch_I_log = VectorXf(3000);
VectorXf rew_log_pitch = VectorXf(3000);
float totes_rew = 0;

void loop() 
{
  if(!control_mode) // PPO  ===========================================================================================================================
  {
    env->m_logrp = 0;
    env->m_logrc = 0;
    env->m_logrk = 0;
    iterReset(); // Used for logging
    
    // Test policy (deterministic) ---------------------------------------------------------------------------------------------------------
  
    env->m_pitch_inv = 1.0f;
    det_manual_time += env->episodeReset(manual_reset);
//    unsigned long stamp1 = millis();
    while(!env->m_done) //(env->m_done == 0 || env->m_done == 2)
    {
      agent->ForwardPropagate(env->m_ob);
      env->markovStep(agent->m_mu, markov_time);
    }
//    unsigned long stamp2 = millis();
//    unsigned long a = stamp2-stamp1;
//    Serial.println(a);
    float non_inv_return = env->getReturn();
    
    sendTorque(0, 0);
//    setEyes(1);
    delay(1000);  //allow human time to move
//    Serial.println(a);
    batch_cum_det += env->getReturn();

    eps_det +=1;
    
    test_time = iter_time - det_wait_time - det_manual_time;

    

    // Collect dataset (stochastic) --------------------------------------------------------------------------------------------------------
    unsigned long t1 = millis();
    setEyes(3);
    
    episode_cum = 0;
    while(agent->m_batch_id < agent->m_desired_batch_size){
      sendTorque(0, 0);
//      delay(200);
      soc_manual_time += env->episodeReset(manual_reset);
      while(!env->m_done)
      {
        agent->ForwardPropagate(env->m_ob, pposas);
        env->markovStep(agent->m_act, markov_time);
        agent->CacheSample(env->m_reward, env->m_done, env->m_ob);
        total_training_steps += 1;
        episode_cum += env->m_reward;  // logging
      }
      sendTorque(0, 0);
      setEyes(3);
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
    

  } else if(control_mode==5) { //  Manually adjust set point for agent over bluetooth =====================================================

    env->episodeReset(false);  //no manual_reset

    int counta = 0;
    while(true){
      agent->ForwardPropagate(env->m_ob);


      /////
      



      /////




      
      

      float tmpY = (incomingReadings.actY + 5.5)/127.5 - 1.0f;
      float tmpX = (incomingReadings.actX + 2.0)/127.5 - 1.0f;
      bool userKill = incomingReadings.actL;
//      env->m_pri_set_point = -tmpY/2.0f;
//      if (counta == 500){
//        env->m_sec_set_point = 0.5;// -tmpY/2.0f;
//      }
      
      turn = -tmpX / 2.0f; 
//      agent->m_mu[0] = 0;//tmpY/10.0f;

      if (counta < 3000){
        pitch_log[counta]       = env->m_ob[0];
        act_log[counta]        = agent->m_mu[0];
        pitch_D_log[counta]    = env->m_ob[1];
        pitch_I_log[counta]    = 0;//env->m_ob[2];
//        rew_log_pitch[counta]  = env->m_logrp;
        totes_rew += env->m_reward;
        rew_log_pitch[counta]  = totes_rew;
      } else if(counta==3000){
        saveLog(pitch_log, act_log, pitch_D_log, pitch_I_log, rew_log_pitch);
        setEyes(2);
      }
      counta += 1;


      env->markovStep(agent->m_mu, markov_time);  //agent->m_mu not m_act


      
     // Serial.print(agent->m_ob[0]); Serial.print('\t'); Serial.print(agent->m_ob[1]); Serial.print('\t'); Serial.print(agent->m_ob[2]); Serial.print('\t'); Serial.println(agent->m_ob[3]);
//      Serial.print('\t'); Serial.print(posL); Serial.print('\t'); Serial.println(posR);
//      Serial.print(incomingReadings.actX);
      
//      Serial.println(tmpX);
    }
    

  
  } else {  // Hardcoded Ziegler Nichols PID ===============================================================================================
    //This is used for classic control model free benchmarking, note consider your markov time

    if(control_mode==1){            // PID controller
       Kp = Ku * 0.6f;
       Ti = Tu / 2.0f;
       Td = Tu / 8.0f;
       Ki = Kp/Ti;
       Kd = Kp*Td;
       
      } else if(control_mode==2){   // PD controller TODO Parameters NEEEDS CHANGING NO INTERNET
       Kp = Ku * 0.8f;
       Ti = 0.0f;
       Td = Tu / 8.0f;
       Ki = 0.0f;
       
      } else if(control_mode==3){   // PI controller
       Kp = Ku * 0.45f;
       Ti = Tu / 1.2f;
       Td = 0.0f;
       Ki = Kp/Ti;
       Kd = Kp*Td;
       
      } else if(control_mode==4){   // P controller
        Kp = Ku * 0.65f;
        Ki = 0;
        Kd = 0;
      }

    StandardPID standPID(0.2, Kp, Ki, Kd);

    standPID.reset();
    int counta = 0;
    env->episodeReset(false);
    while(1){
      standPID.update(env->m_pri_true, 0);
      agent->m_act[0] = standPID.update(env->m_pri_true, 0);


      if (counta < 3000){
        pitch_log[counta]       = env->m_ob[0];
        act_log[counta]        = agent->m_act[0];
        pitch_D_log[counta]    = env->m_ob[1];
        pitch_I_log[counta]    = 69;// env->m_ob[2];
        totes_rew += env->m_reward;
        rew_log_pitch[counta]  = totes_rew;
      } else if(counta==3000){
        saveLog(pitch_log, act_log, pitch_D_log, pitch_I_log, rew_log_pitch);
        setEyes(2);
      }
      counta += 1;

      
      
//      Serial.println(env->m_cur_cart_pos);
      env->markovStep(agent->m_act, markov_time);
    }
  }
}
