#ifndef AGENT_H
#define AGENT_H
#include <random>
#include "NN.hpp"

#include <ArduinoEigen.h>
using Eigen::MatrixXf;
using Eigen::VectorXf;

/**
 *  Agent Class
 */
class Agent
{
public:
    
    ~Agent() = default;

    float m_standard_deviation = 0.5;
    float m_min_standard_deviation = 0.3;
    float m_anneal_rate = 0.0003;
    float m_value_hat;
    std::default_random_engine m_generator;
    float m_lr;
    uint16_t m_seed;
    float m_log_prob;
    bool m_load_network;
    bool m_adam;
    uint16_t m_done_counter;

    // Actor architecture dimentions
    uint16_t m_in_size;
    uint16_t m_act_size;
    uint16_t m_h1_size;
    uint16_t m_h2_size;
    uint16_t m_desired_batch_size;
    uint16_t m_step_limit;
    uint16_t m_minibatch_size; //have user specified. must be less than m_batch_id

    std::uint16_t m_batch_id = 0; // number in batch (i.e. sample in iteration)
    float ip_clip = 0.2; //have user specified
    float m_avg_eps_reward_cumsum = 0;  //undiscounted
    
    NeuralNetwork mNet;
    NeuralNetwork vNet;
    NeuralNetwork pNet;


    // Actor mean output and action sample
    Eigen::VectorXf m_mu  = Eigen::VectorXf(m_act_size);
    Eigen::VectorXf m_act = Eigen::VectorXf(m_act_size);
    Eigen::VectorXf m_act_prev = Eigen::VectorXf::Zero(m_act_size); // only used for PPO Smoothing Action Space
    
    Eigen::VectorXf m_ob = Eigen::VectorXf(m_in_size);
    
    
    // Cache from agent: mu, sigma, value_hat, log_likelihood(action_taken| mu, sigma), action_taken, done, model_hat
    uint16_t max_samples = m_desired_batch_size + m_step_limit + 1;  //big enough buffer, plus 1 for gae which isnt used
    Eigen::MatrixXf m_mu_cache = Eigen::MatrixXf(max_samples, m_act_size);
    Eigen::VectorXf m_standard_deviation_cache = Eigen::VectorXf(max_samples);
    Eigen::VectorXf m_value_hat_cache = Eigen::VectorXf(max_samples);
    Eigen::VectorXf m_log_prob_cache = Eigen::VectorXf(max_samples);
    Eigen::MatrixXf m_act_cache = Eigen::MatrixXf(max_samples, m_act_size);
    Eigen::MatrixXf m_model_hat_cache = Eigen::MatrixXf(max_samples, m_in_size);
    
    // Cache from environment: ob, rew, done. Note not next ob
    Eigen::MatrixXf m_ob_cache = Eigen::MatrixXf(max_samples, m_in_size);
    Eigen::MatrixXf m_next_ob_cache = Eigen::MatrixXf(max_samples, m_in_size);
    Eigen::VectorXf m_reward_cache = Eigen::VectorXf(max_samples);
    Eigen::VectorXi m_done_cache = Eigen::VectorXi(max_samples);
    
    // Cache generated from rewards and value estrimates
    Eigen::VectorXf m_return_cache = Eigen::VectorXf(max_samples);
    Eigen::VectorXf m_adv_cache = Eigen::VectorXf(max_samples);
    
    
    // Random Agent
    Agent(uint16_t in_size,
          uint16_t act_size,
          uint16_t h1_size = 4,
          uint16_t h2_size = 4,
          float lr = float(0.0003),
          uint16_t seed = 1,
          uint16_t desired_batch_size=2000,
          uint16_t minibatch_size=64,
          uint16_t step_limit=200,
          bool load_network=false,
          bool adam=true);


   void AssignElements();
    
    void ForwardPropagate(Eigen::VectorXf env_ob, int pposas=false); //false disabled, if true smoother action transition with effecting POMDP;
    
    float GaussianLikelihood1D(float mu, float sigma, float x);
    
    bool CacheSample(float reward, int done, Eigen::VectorXf next_ob);
    
    void ClearSamples();
    
    void ReturnGAE(float gamma=0.99, float lambda=0.95);
    
    bool UpdatePPO(std::uint16_t n_epochs);
    
    void BackPropagate(Eigen::VectorXf ob, Eigen::VectorXf act, float ret, float adv, float old_log_prob, bool clipped, bool zero_grad_buffer);
    
    void ZeroGradientBuffer();  // move to private
    uint16_t m_buffer_count;  // move to private
    
    void AddGradientsToBuffer(bool zero_buffer);
    
    void AverageGradientBuffer();
    
    void AdamUpdate(float b1=0.9, float b2=0.999, float b1v=0.9, float b2v=0.999, float e_small=float(1e-7));  // move to private
    
    void VanillaUpdate();  // move to private
    
    void ZeroAdamMomentums();  // move to private
    
    void LinearAnneal();
    

    void SaveNetworkSD(const char * folder, int iter);

    void LoadNetworkSD(const char * folder, int iter);
    
   private:
   
    void SaveMDPDataSD(const char * folder, int iter, bool allData=false);
   
   
    
};

#endif // AGENT_H
