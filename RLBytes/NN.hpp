#ifndef NN_H
#define NN_H

#include "MemorySD.hpp" 
#include <random>
#include <ArduinoEigen.h>
using Eigen::MatrixXf;
using Eigen::VectorXf;

/**
 *  `Simple 2 hidden layer NeuralNetwork class
 */
class NeuralNetwork
{
public:
    std::uint16_t m_seed;
    std::default_random_engine m_generator;
    float m_lr;
    bool m_adam;
    bool m_load;
    bool m_ascent;
    
    // Actor architecture dimentions
    std::uint16_t m_in_size;
    std::uint16_t m_out_size;
    std::uint16_t m_h1_size;
    std::uint16_t m_h2_size;
    std::uint16_t m_batch_size;
    std::uint16_t m_minibatch_size;
    
    std::uint16_t m_buffer_count;
    std::uint32_t m_adam_count;
    MatrixXf m_x;
    MatrixXf m_y;
    MatrixXf m_x_train;
    MatrixXf m_y_train;
    MatrixXf m_x_test;
    MatrixXf m_y_test;
    
    VectorXf m_in = VectorXf(m_in_size);
    VectorXf m_out  = VectorXf(m_out_size);
    
    
    // Weights and Bias
    MatrixXf m_W1 = MatrixXf(m_h1_size, m_in_size);
    MatrixXf m_W2 = MatrixXf(m_h2_size, m_h1_size);
    MatrixXf m_W3 = MatrixXf(m_out_size, m_h2_size);
    VectorXf m_B1 = VectorXf(m_h1_size);
    VectorXf m_B2 = VectorXf(m_h2_size);
    VectorXf m_B3 = VectorXf(m_out_size);
    
    // Grads
    MatrixXf m_dW1 = MatrixXf(m_h1_size, m_in_size);
    MatrixXf m_dW2 = MatrixXf(m_h2_size, m_h1_size);
    MatrixXf m_dW3 = MatrixXf(m_out_size, m_h2_size);
    VectorXf m_dB1 = VectorXf(m_h1_size);
    VectorXf m_dB2 = VectorXf(m_h2_size);
    VectorXf m_dB3 = VectorXf(m_out_size);
    
    // Grad Buffer
    MatrixXf m_dW1_buf = MatrixXf(m_h1_size, m_in_size);
    MatrixXf m_dW2_buf = MatrixXf(m_h2_size, m_h1_size);
    MatrixXf m_dW3_buf = MatrixXf(m_out_size, m_h2_size);
    VectorXf m_dB1_buf = VectorXf(m_h1_size);
    VectorXf m_dB2_buf = VectorXf(m_h2_size);
    VectorXf m_dB3_buf = VectorXf(m_out_size);
    
    // Adam grad sums
    MatrixXf m_dW1_mom;
    MatrixXf m_dW2_mom;
    MatrixXf m_dW3_mom;
    VectorXf m_dB1_mom;
    VectorXf m_dB2_mom;
    VectorXf m_dB3_mom;
    
    MatrixXf m_dW1_mom2;
    MatrixXf m_dW2_mom2;
    MatrixXf m_dW3_mom2;
    VectorXf m_dB1_mom2;
    VectorXf m_dB2_mom2;
    VectorXf m_dB3_mom2;
    
    // Actor Pre activation
    VectorXf m_Z1 = VectorXf(m_h1_size);
    VectorXf m_Z2 = VectorXf(m_h2_size);
    VectorXf m_Z3 = VectorXf(m_out_size);
    
    VectorXf m_dZ1 = VectorXf(m_h1_size);
    VectorXf m_dZ2 = VectorXf(m_h2_size);
    VectorXf m_dZ3 = VectorXf(m_out_size);
    
    // Actor Post activation
    VectorXf m_A1 = VectorXf(m_h1_size);
    VectorXf m_A2 = VectorXf(m_h2_size);
    VectorXf m_A3 = VectorXf(m_out_size);
    
    VectorXf m_dA1 = VectorXf(m_h1_size);
    VectorXf m_dA2 = VectorXf(m_h2_size);
    VectorXf m_dA3 = VectorXf(m_out_size);

    //Probe for debug only
    Eigen::VectorXf m_probe = Eigen::VectorXf(49);

    NeuralNetwork(uint16_t in_size,
                  uint16_t out_size,
                  uint16_t h1_size = 8,
                  uint16_t h2_size = 8,
                  float lr = 0.0003f,
                  uint16_t seed = 1,
                  uint16_t minibatch_size = 64,
                  bool adam=true,
                  bool ascent=false);
    
    void WeightInit(float mu = 0.0f,
                    float stand_dev = 1.0f,
                    const float initial_bias=0.01);
    
    ~NeuralNetwork() = default;
    void ForwardPropagate();
    void TanhActivation(VectorXf& Z, VectorXf& A);
    VectorXf TanhDerivative(Eigen::VectorXf& Z);
    void BackPropagate(Eigen::VectorXf label_,
                       bool zero_grad_buffer,
                       float ppo_corrector=1.0);  // 1.0 is no correction
    
    void ZeroGradientBuffer();
     void AddGradientsToBuffer(bool zero_buffer);
    
    void AverageGradientBuffer();
    
    void VanillaUpdate();
    
    void ZeroAdamMomentums();
    
    void AdamUpdate(
                    float b1=0.9,
                    float b2=0.999,
                    float e_small=float(1e-7));
    void PrintWeights();

    void SaveNetworkSD(const char * folder, const int iter, const char * xFile);
    void LoadNetworkSD(const char * folder, const int iter, const char * xFile);

    void ProbeNetwork();
    
    void WeightInverse();
};

#endif // NN_H
