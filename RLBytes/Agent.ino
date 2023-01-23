#include "Agent.hpp"

Agent::Agent(uint16_t in_size,
             uint16_t act_size,
             uint16_t h1_size,
             uint16_t h2_size,
             float lr,
             uint16_t seed,
             uint16_t desired_batch_size,
             uint16_t minibatch_size,
             uint16_t step_limit,
             bool load_network,
             bool adam)
:m_in_size(in_size),
m_act_size(act_size),
m_h1_size(h1_size),
m_h2_size(h2_size),
m_lr(lr),
m_seed(seed),
m_desired_batch_size(desired_batch_size),
m_minibatch_size(minibatch_size),
m_step_limit(step_limit),
m_load_network(load_network),
m_adam(adam),
vNet(
         m_in_size,
         1,
         m_h1_size,
         m_h2_size,
         m_lr,
         m_seed,
         m_minibatch_size,
         m_adam,
         false  //false is descent
         ),
mNet(
         m_act_size+m_in_size,
         m_in_size,
         m_h1_size,
         m_h2_size,
         m_lr,
         m_seed,
         m_minibatch_size,
         m_adam,
         false
         ),
pNet(
         m_in_size,
         m_act_size,
         m_h1_size,
         m_h2_size,
         m_lr,
         m_seed,
         m_minibatch_size,
         m_adam,
         true
         )
             {
                Serial.print("Agent created: max_samples agent ");
                  Serial.println(max_samples);
                  
                  pNet.ProbeNetwork();
                  vNet.ProbeNetwork();  
                  ZeroAdamMomentums();   
               
             }

// Forward -----------------------------------------------------------------------------------------
void Agent::ForwardPropagate(VectorXf env_ob, int pposas){
    m_ob = env_ob;
    
    // Value Model prediction
    vNet.m_in = m_ob;
    vNet.ForwardPropagate();
    
    // Action Model mu and sampling
    pNet.m_in = m_ob;
    pNet.ForwardPropagate();
    m_mu = pNet.m_A3;
    m_value_hat = vNet.m_A3(0);
    
    // Sample each action dimension.
    // Assumption: standard deviation same for all dimentions
    for(std::uint16_t i=0; i<m_act_size; ++i){
        std::normal_distribution<double> distribution(m_mu(i), m_standard_deviation);
        m_act(i) = distribution(m_generator);
    }

    // PPOSAS ========================================
// This needs to be disabled for updating only in traing is it to be used 
    if (pposas==true){
      for(std::uint16_t i=0; i<m_act_size; ++i){
        m_act(i) = (m_act(i) + m_act_prev(i))*0.5f;
      }
    }
    m_act_prev = m_act;
    //===============================================
    
    
    // Find gaussian log likelihood of m_act
    Eigen::VectorXf prob_breakdown = Eigen::VectorXf(m_act_size);
    for(std::uint16_t i=0; i<m_act_size; ++i){
      prob_breakdown(i) = GaussianLikelihood1D(m_mu(i), m_standard_deviation, m_act(i));
    }
    m_log_prob = log(prob_breakdown.prod());  //check base
    
    // Dynamics Model prediction
    for(uint16_t ii=0; ii < m_in_size; ii++){mNet.m_in[ii]=m_ob[ii];}
    for(uint16_t ii=m_in_size; ii < m_in_size+m_act_size; ii++){mNet.m_in[ii] = m_act[ii-m_in_size];}
    mNet.ForwardPropagate();
}

// Likelihood --------------------------------------------------------------------------------------
float Agent::GaussianLikelihood1D(float mu, float sigma, float x){
    float likelihood = (exp(-0.5f*(pow(x-mu, 2))/(pow(sigma, 2))))/sqrt(M_PI*2.0f*pow(sigma, 2));
    return likelihood;
}

// Cache Sample -------------------------------------------------------------------------------------
bool Agent::CacheSample(float reward, int done, Eigen::VectorXf next_ob){
    /*
     Saves sample set
    ----------------------------------------
     m_mu (vector of act_size),
     m_standard_deviation (scalar for now),
     m_value_hat predicted value_hat of observation (scalar),
     m_log_prob (scalar),
     m_act (vector of act size)
     m_done (scalar)
     */
    if(m_batch_id > max_samples-1) {
        Serial.println("RL Error: Exceed sample buffer, this sample wasnt added");
        return true;
    } else {
        m_mu_cache.row(m_batch_id) = m_mu;
        m_standard_deviation_cache(m_batch_id) = m_standard_deviation;
        m_value_hat_cache(m_batch_id) = m_value_hat;
        m_log_prob_cache(m_batch_id) = m_log_prob;
        m_act_cache.row(m_batch_id) = m_act;
        m_done_cache(m_batch_id) = done;
        m_reward_cache(m_batch_id) = reward;
        m_ob_cache.row(m_batch_id) = m_ob;
        m_next_ob_cache.row(m_batch_id) = next_ob;
        
        m_batch_id +=1;  // ToDO add reset  //todo to big think addding one og null
        return false;
    }
}

// Find GAE and Returns ----------------------------------------------------------------------------
void Agent::ReturnGAE(float gamma, float lambda){
    /*
     ref GAE paper: https//arxiv.org/pdf/1506.02438.pdf
     */
    
    // Extract batch from buffer
    
    auto n_samples                     = m_batch_id;

    Eigen::VectorXf return_batch       = Eigen::VectorXf::Zero(n_samples);
    Eigen::VectorXf adv_batch          = Eigen::VectorXf::Zero(n_samples+1);
    // note last value will never be used as assumes ends in a terminal state which is defines as zero value.
    // For early stopping add in a feed forward final state in NN.
    
    Eigen::VectorXf reward_batch       = m_reward_cache(Eigen::seqN(0, n_samples));
    Eigen::VectorXi done_batch         = m_done_cache(Eigen::seqN(0, n_samples));
    Eigen::VectorXf val_hat_batch      = m_value_hat_cache(Eigen::seqN(0, n_samples+1));
    val_hat_batch[n_samples] = 0;      // note this value will never be used as ends in a terminal state
    Serial.print("n samples");
    Serial.println(n_samples);
    
    // Compute GAE Advantages and Return
    for (std::int32_t i=n_samples-1; i>-1; --i )
    {
        float delta;
        
        // Early stopping on a non-Terimal state
        if(done_batch[i]==2){
            Eigen::VectorXf next_ob = m_next_ob_cache.row(i);
            ForwardPropagate(next_ob);
            float death_value = m_value_hat;
            delta = reward_batch[i] + gamma * death_value - val_hat_batch[i];
        
        // True Terimal state
        } else if(done_batch[i]==1){
            delta = reward_batch[i] - val_hat_batch[i];
            
        // Non-Terminal state
        } else {
            delta = reward_batch[i] + gamma * val_hat_batch[i+1] - val_hat_batch[i];
        }
        
        adv_batch[i] = delta + gamma * lambda * adv_batch[i + 1] * (not done_batch[i]);
    }
    
    adv_batch = adv_batch(Eigen::seqN(0, n_samples));
    val_hat_batch = val_hat_batch(Eigen::seqN(0, n_samples));
    return_batch = val_hat_batch + adv_batch;
    
    //ToDo shorten returns and adv by 1
    
    
    // Normalise Advantages
    float adv_mean = adv_batch.mean();
    float adv_std = 0;
    for (std::int32_t i=0; i < n_samples; ++i){
        adv_std += pow(adv_batch[i] - adv_mean, 2.0f);
    }
    adv_std = std::sqrt( adv_std / ( (float(n_samples)-1) + float(1e-8)) );  //unbiased std estimate and avoid NaNs with 1e-8
    
    // set Mean=0, and Standard Deviation=1
    for (std::int32_t i=0; i < m_batch_id; ++i){
        adv_batch[i] = (adv_batch[i] - adv_mean) / (adv_std + float(1e-8));
    }
    m_return_cache = return_batch;
    m_adv_cache = adv_batch;
    
    // User logging only
    m_avg_eps_reward_cumsum = 0;
    std::uint32_t n_episodes = 0;
    for (std::int32_t i=0; i < m_batch_id; ++i){
        if (done_batch[i] == true) n_episodes +=1;
        m_avg_eps_reward_cumsum += reward_batch[i];
    }
    m_avg_eps_reward_cumsum /= float(n_episodes);
};


// Update PPO ----------------------------------------------------------------------------
bool Agent::UpdatePPO(std::uint16_t n_epochs){

    // Compute GAE and Normalise
    ReturnGAE();
    uint32_t clipped_count = 0;
    uint32_t unclipped_count = 0;
    uint32_t updates_per_epoch_count = 0;
    
    // Compute min_batch sizes
    std::uint16_t n_minibatches = int(ceil(m_batch_id / m_minibatch_size));
    std::uint16_t final_minibatch_size;
    if (n_minibatches==0){
        Serial.println("RLError: n_minibatches is 0");
        return true;
    }
    if ((m_batch_id % m_minibatch_size) !=0) {
        final_minibatch_size = m_batch_id % m_minibatch_size;
        n_minibatches +=1;
    } else {
        final_minibatch_size = m_minibatch_size;
    }
    
    for(std::int16_t e=0; e < n_epochs; ++e){
        // Shuffle batch_ids
        auto IDs = Eigen::VectorXi(m_batch_id);
        for (std::uint16_t i=0; i < m_batch_id; ++i){IDs[i] = i;}
        shuffle (IDs.begin(), IDs.end(), m_generator);
        

        for(std::int16_t m=0; m < n_minibatches; ++m){
            Eigen::VectorXi mini_IDs;
            std::uint16_t start;
            std::uint16_t end;
            
            if(m==(n_minibatches-1)){
                start = m * m_minibatch_size;
                end = m * m_minibatch_size + final_minibatch_size;
            } else {
                start = m * m_minibatch_size;
                end = m * m_minibatch_size + m_minibatch_size;
            }
            mini_IDs = IDs(Eigen::seqN(start,end-start));

            for(std::int16_t i=0; i < mini_IDs.size(); ++i){
                std::uint16_t id = mini_IDs[i];
                // Extract sample
                m_ob = m_ob_cache.row(id);  // ob prior to act
                Eigen::VectorXf act = m_act_cache.row(id);
                float old_log_prob = m_log_prob_cache(id);
                float ret = m_return_cache(id);
                float adv = m_adv_cache(id);
                
                // Compute new log likelihood with old action and new value will latest mini update.
                // By old I mean action from data collection stage
                ForwardPropagate(m_ob);  // takes in m_ob and updates m_mu, m_standard_deviation, m_value_hat, m_A3, m_A2m ect
                Eigen::VectorXf prob_breakdown = Eigen::VectorXf(m_act_size);
                for(std::uint16_t ii=0; ii<m_act_size; ++ii){
                  prob_breakdown(ii) = GaussianLikelihood1D(m_mu(ii), m_standard_deviation, act(ii)); //Note collection act NOT new m_act
                }
                float new_log_prob = log(prob_breakdown.prod());
                
                // Perform Importants Sampling and Clipping
                float likelihood_ratio = exp(new_log_prob-old_log_prob);
                float surrogate1 = likelihood_ratio * adv;
                float surrogate2 = constrain(likelihood_ratio, 1.0f - ip_clip, 1.0f + ip_clip) * adv;
                
                // Select Surrogate loss (Actor)
                float actor_loss;
                bool clipped;
                if (surrogate2 < surrogate1){
                    actor_loss = surrogate2;
                    clipped = true;
                    clipped_count += 1;
                } else {
                    actor_loss = surrogate1;
                    clipped = false;
                    unclipped_count += 1;
                }
                // Backpropergate and Compute Vanilla gradients for just the one sample (But don't update yet, juat store grads)
                BackPropagate(m_ob, act, ret, adv, old_log_prob, clipped, not i);  // updates grads and adds to grad buffer. Zeros grads if first in minbatch. Does not update weight yet
                updates_per_epoch_count +=1;
            }
            
            // Find average grads in the buffer
            AverageGradientBuffer();
            
            // Update weights with average grads from buffer
            (m_adam) ? AdamUpdate() : VanillaUpdate();
        }
    }
    m_batch_id = 0;
    return false;
};

// BackPropagate ----------------------------------------------------------------------------
void Agent::BackPropagate(Eigen::VectorXf ob,
                          Eigen::VectorXf act,
                          float ret,
                          float adv,
                          float old_log_prob,
                          bool clipped,
                          bool zero_grad_buffer){
    
    Eigen::VectorXf ret_vec = Eigen::VectorXf(1);
    ret_vec(0) = ret;
    vNet.BackPropagate(ret_vec, zero_grad_buffer);
    
    float ppo_corrector = (1.0f - float(clipped)) * (adv / old_log_prob);  // zero gradients if clipped
    pNet.BackPropagate(act, zero_grad_buffer, ppo_corrector);
};


// Adam -----------------------------------------------------------------------------------
void Agent::AdamUpdate(float b1, float b2, float b1v, float b2v, float e_small){
    vNet.AdamUpdate(b1v, b2v);
    pNet.AdamUpdate(b1, b2);
}

// Valilla --------------------------------------------------------------------------------
void Agent::VanillaUpdate(){

    vNet.VanillaUpdate();
    pNet.VanillaUpdate();
}

// Linear Anneal ---------------------------------------------------------------------------
void Agent::LinearAnneal(){
    if (m_standard_deviation > m_min_standard_deviation) {m_standard_deviation -= m_anneal_rate;}
}

// Zero grads in buffer ---------------------------------------------------------------------------
void Agent::ZeroGradientBuffer(){
    vNet.ZeroGradientBuffer();
    pNet.ZeroGradientBuffer();
};

// Zero Momemtums ---------------------------------------------------------------------------------
void Agent::ZeroAdamMomentums(){
    vNet.ZeroAdamMomentums();
    pNet.ZeroAdamMomentums();
}

// Add grads to buffer ---------------------------------------------------------------------------
void Agent::AddGradientsToBuffer(bool zero_buffer){
    vNet.AddGradientsToBuffer(zero_buffer);
    pNet.AddGradientsToBuffer(zero_buffer);
}

// Average grads in buffer -----------------------------------------------------------------------
void Agent::AverageGradientBuffer(){
    vNet.AverageGradientBuffer();
    pNet.AverageGradientBuffer();
}

// Save Network -----------------------------------------------------------------------------------
void Agent::SaveNetworkSD(const char * folder, int iter){
  pNet.ProbeNetwork();
  vNet.ProbeNetwork();
  
//  if(iter==0){
//    createDir(SD, folder);
//    Serial.println("folder made");
//  }
  char iFile[100] = "/info_";
  char mFile[100] = "/model_";
  char dFile[100] = "/data_";
  char vFile[100] = "/value_";
  char pFile[100] = "/policy_";

  pNet.SaveNetworkSD(folder, iter, pFile);
  Serial.println(folder);
  vNet.SaveNetworkSD(folder, iter, vFile);
  Serial.println(folder);
  mNet.SaveNetworkSD(folder, iter, mFile);
  Serial.println(folder);
  SaveMDPDataSD(folder, iter);
  Serial.println(folder);
}

// Load Network -----------------------------------------------------------------------------------
void Agent::LoadNetworkSD(const char * folder, int iter){

  char mFile[100] = "/model_";
  char vFile[100] = "/value_";
  char pFile[100] = "/policy_";

  pNet.LoadNetworkSD(folder, iter, pFile);
  vNet.LoadNetworkSD(folder, iter, vFile);
  mNet.LoadNetworkSD(folder, iter, mFile);

  pNet.ProbeNetwork();
  vNet.ProbeNetwork(); 
}

// Save MDP Data ------------------------------------------------------------------------------------
void Agent::SaveMDPDataSD(const char * folder, const int iter, bool allData){
  
    char file[100] = "/mdp_";
    char str_iter[5];
    sprintf(str_iter, "%d", iter);
    char xPath[100];
    strcpy(xPath, folder);
    strcat(xPath, file);
    strcat(xPath, str_iter);
    strcat(xPath, ".json");


    nlohmann::json jm;
    jm = nlohmann::json::parse(R"([])");

    int a_size = m_act_size;
    int e_size = m_in_size;
    Eigen::VectorXf x(a_size+e_size);
    for(uint16_t i=0; i < m_batch_id; i++)
    {
        for(uint16_t ii = 0; ii < e_size; ii++){x[ii] = m_ob_cache.row(i)[ii];}
        for(uint16_t ii = e_size; ii < e_size+a_size; ii++){x[ii] = m_act_cache.row(i)[ii-e_size];}  
            jm[0]["x"][i] = x;
            jm[0]["y"][i] = m_next_ob_cache.row(i);
    }

    std::string serializedObject = jm.dump(4);
    writeFile(SD, xPath, serializedObject.c_str()); 
}
