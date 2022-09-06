#include "RLBytes.hpp"

// Constructor  ------------------------------------------------------------------------------------------
NeuralNetwork::NeuralNetwork(std::uint16_t in_size,
                             std::uint16_t out_size,
                             std::uint16_t h1_size,
                             std::uint16_t h2_size,
                             float lr,
                             std::uint16_t seed,
                             std::uint16_t minibatch_size,
                             bool adam,
                             bool ascent)
:m_in_size(in_size),
m_out_size(out_size),
m_h1_size(h1_size),
m_h2_size(h2_size),
m_lr(lr),
m_seed(seed),
m_minibatch_size(minibatch_size),
m_adam(adam),
m_ascent(ascent)

{
    WeightInit();
    ZeroAdamMomentums();
};

// Weight Init ------------------------------------------------------------------------------------------
void NeuralNetwork::WeightInit(float mu, float stand_dev, const float initial_bias){
  /*
     Builds a Gaussian Distribution with mu and stand_dev
     Generates weights from distribution
     Adds Xavier initialisation as the activsation function is tanh
     Sets bias to inital bias
     */
    
    // Set generator and seed
//    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
    std::normal_distribution<double> distribution(mu, stand_dev);
    
    // Generatr Weights
    for(std::uint16_t i=0; i<m_W1.size(); ++i){m_W1(i) = distribution(m_generator);}
    for(std::uint16_t i=0; i<m_W2.size(); ++i){m_W2(i) = distribution(m_generator);}
    for(std::uint16_t i=0; i<m_W3.size(); ++i){m_W3(i) = distribution(m_generator);}
    
    // Xavier Init on Weights as the NN uses Tanh activation function
    m_W1 *= sqrt(double(1)/double(m_in_size));
    m_W2 *= sqrt(double(1)/double(m_h1_size));
    m_W3 *= sqrt(double(1)/double(m_h2_size));
    
    // Init Biases
    m_B1.setConstant(initial_bias);
    m_B2.setConstant(initial_bias);
    m_B3.setConstant(initial_bias);
}

// Forward --------------------------------------------------------------------------------------------
void NeuralNetwork::ForwardPropagate(){ 
  for(int i = 0; i<8; i++){
  }
  m_Z1 = (m_W1 * m_in) + m_B1;  // dot product then element addition
  TanhActivation(m_Z1, m_A1);   // tanh
  
  m_Z2 = m_W2 * m_A1 + m_B2;   // dot product then element addition
  TanhActivation(m_Z2, m_A2);  // tanh
  
  m_Z3 = m_W3 * m_A2 + m_B3;   // dot product then element addition
  m_A3 = m_Z3;                 // Linear activation only. m_A3 is also y_hat
  
  m_out = m_A3;                // means of each dimention
  
}
// BackProp --------------------------------------------------------------------------------------------
void NeuralNetwork::BackPropagate(Eigen::VectorXf label_,
                                  bool zero_grad_buffer,
                                  float ppo_corrector){
    // This assumes just done forward propergation on this sample!
    
    m_dZ3 = (m_A3 - label_) * ppo_corrector;

    // Weights 3
    m_dW3 = m_dZ3 * m_A2.transpose();                                       //dot product
    m_dB3 = m_dZ3;
    m_dA2 = m_dZ3.transpose() * m_W3;                                       //dot product
    m_dZ2 = m_dA2.cwiseProduct(TanhDerivative(m_Z2));                       //coefficent wise
    
    // Weights 2
    m_dW2 = m_dZ2 * m_A1.transpose();                                       //dot product
    m_dB2 = m_dZ2;
    m_dA1 = m_dZ2.transpose() * m_W2;                                       //dot product
    m_dZ1 = m_dA1.cwiseProduct(TanhDerivative(m_Z1));
    
    // Weight 1
    m_dW1 = m_dZ1 * m_in.transpose();                                         //dot product
    m_dB1 = m_dZ1;
    
    AddGradientsToBuffer(zero_grad_buffer); // Adds gradients to buffer. Clears first if true
}

// Activation ------------------------------------------------------------------------------------------
void NeuralNetwork::TanhActivation(Eigen::VectorXf& Z, Eigen::VectorXf& A){
    float clip_limit = 7.6;
    for(std::uint16_t i=0; i<Z.size(); ++i){
        // Clip to stop infs
        if(Z(i)>clip_limit){
            Z(i)=clip_limit;
        }else if (Z(i)<-clip_limit){
            Z(i)= -clip_limit;
        }
        // hipster tanh element wise
        A(i) = (exp(Z(i)) - exp(-Z(i))) / (exp(Z(i)) + exp(-Z(i)));
    }
}

// Activation Derivative --------------------------------------------------------------------------------
Eigen::VectorXf NeuralNetwork::TanhDerivative(Eigen::VectorXf& Z) {
    Eigen::VectorXf AA = Eigen::VectorXf(Z.size());
    // Quotient Rule
    TanhActivation(Z, AA);  // updates Z and A
    for(std::uint16_t i=0; i<Z.size(); ++i){
        AA(i) = 1.0f - pow(AA(i), 2.0f);  //TODO check not minus
    }
    return AA;
}

// Vanilla gradient descent/ascent ----------------------------------------------------------------------
void NeuralNetwork::VanillaUpdate(){
    float direction = -1;
    if(m_ascent){direction *= -1;}
    
    m_W1 = m_W1.array() + direction * m_lr * m_dW1_buf.array();
    m_W2 = m_W2.array() + direction * m_lr * m_dW2_buf.array();
    m_W3 = m_W3.array() + direction * m_lr * m_dW3_buf.array();
    
    m_B1 = m_B1.array() + direction * m_lr * m_dB1_buf.array();
    m_B2 = m_B2.array() + direction * m_lr * m_dB2_buf.array();
    m_B3 = m_B3.array() + direction * m_lr * m_dB3_buf.array();
}

// Adam gradient descent/ascent ----------------------------------------------------------------------
void NeuralNetwork::AdamUpdate(
                               float b1,
                               float b2,
                               float e_small){
    
    float direction = -1;
    if(m_ascent){direction *= -1;}
    
    m_dW1_mom = b1 * m_dW1_mom.array() + (1.0f - b1) * m_dW1_buf.array();
    m_dW2_mom = b1 * m_dW2_mom.array() + (1.0f - b1) * m_dW2_buf.array();
    m_dW3_mom = b1 * m_dW3_mom.array() + (1.0f - b1) * m_dW3_buf.array();
    m_dB1_mom = b1 * m_dB1_mom.array() + (1.0f - b1) * m_dB1_buf.array();
    m_dB2_mom = b1 * m_dB2_mom.array() + (1.0f - b1) * m_dB2_buf.array();
    m_dB3_mom = b1 * m_dB3_mom.array() + (1.0f - b1) * m_dB3_buf.array();
    
    m_dW1_mom2 = b2 * m_dW1_mom2.array() + (1.0f - b2) * m_dW1_buf.array().pow(2);
    m_dW2_mom2 = b2 * m_dW2_mom2.array() + (1.0f - b2) * m_dW2_buf.array().pow(2);
    m_dW3_mom2 = b2 * m_dW3_mom2.array() + (1.0f - b2) * m_dW3_buf.array().pow(2);
    m_dB1_mom2 = b2 * m_dB1_mom2.array() + (1.0f - b2) * m_dB1_buf.array().pow(2);
    m_dB2_mom2 = b2 * m_dB2_mom2.array() + (1.0f - b2) * m_dB2_buf.array().pow(2);
    m_dB3_mom2 = b2 * m_dB3_mom2.array() + (1.0f - b2) * m_dB3_buf.array().pow(2);

    
    m_W1 = m_W1.array() + direction * m_lr * m_dW1_mom.array() / (m_dW1_mom2.array().pow(0.5) + e_small);
    m_W2 = m_W2.array() + direction * m_lr * m_dW2_mom.array() / (m_dW2_mom2.array().pow(0.5) + e_small);
    m_W3 = m_W3.array() + direction * m_lr * m_dW3_mom.array() / (m_dW3_mom2.array().pow(0.5) + e_small);
    
    m_B1 = m_B1.array() + direction * m_lr * m_dB1_mom.array() / (m_dB1_mom2.array().pow(0.5) + e_small);
    m_B2 = m_B2.array() + direction * m_lr * m_dB2_mom.array() / (m_dB2_mom2.array().pow(0.5) + e_small);
    m_B3 = m_B3.array() + direction * m_lr * m_dB3_mom.array() / (m_dB3_mom2.array().pow(0.5) + e_small);
    
    m_adam_count +=1;
}

// Zero Momemtums ---------------------------------------------------------------------------------
void NeuralNetwork::ZeroAdamMomentums(){
    
    m_adam_count = 0;
    
    m_dW1_mom.setZero(m_h1_size, m_in_size);
    m_dW2_mom.setZero(m_h2_size, m_h1_size);
    m_dW3_mom.setZero(m_out_size, m_h2_size);
    m_dB1_mom.setZero(m_h1_size);
    m_dB2_mom.setZero(m_h2_size);
    m_dB3_mom.setZero(m_out_size);
    
    m_dW1_mom2.setZero(m_h1_size, m_in_size);
    m_dW2_mom2.setZero(m_h2_size, m_h1_size);
    m_dW3_mom2.setZero(m_out_size, m_h2_size);
    m_dB1_mom2.setZero(m_h1_size);
    m_dB2_mom2.setZero(m_h2_size);
    m_dB3_mom2.setZero(m_out_size);
}

// Add grads to buffer ---------------------------------------------------------------------------
void NeuralNetwork::AddGradientsToBuffer(bool zero_buffer){
    
    if (zero_buffer) {
        ZeroGradientBuffer();
        m_buffer_count = 0;
    }
    m_dW1_buf += m_dW1;
    m_dW2_buf += m_dW2;
    m_dW3_buf += m_dW3;
    m_dB1_buf += m_dB1;
    m_dB2_buf += m_dB2;
    m_dB3_buf += m_dB3;
    m_buffer_count += 1;
}

// Average grad buffer --------------------------------------------------------------------------
void NeuralNetwork::AverageGradientBuffer(){

    m_dW1_buf /= float(m_buffer_count);
    m_dW2_buf /= float(m_buffer_count);
    m_dW3_buf /= float(m_buffer_count);
    m_dB1_buf /= float(m_buffer_count);
    m_dB2_buf /= float(m_buffer_count);
    m_dB3_buf /= float(m_buffer_count);
}

// Zero grad buffer ----------------------------------------------------------------------------
void NeuralNetwork::ZeroGradientBuffer(){

    m_dW1_buf.setZero(m_h1_size, m_in_size);
    m_dW2_buf.setZero(m_h2_size, m_h1_size);
    m_dW3_buf.setZero(m_out_size, m_h2_size);
    m_dB1_buf.setZero(m_h1_size);
    m_dB2_buf.setZero(m_h2_size);
    m_dB3_buf.setZero(m_out_size);
};

// Debug  --------------------------------------------------------------------------------------
void NeuralNetwork::PrintWeights(){
  const int precision = 6;
  Serial.println("W1---------------------------------");
  for(std::uint16_t i=0; i<m_h1_size; ++i){
    Serial.println();
    for(std::uint16_t ii=0; ii<m_in_size; ++ii){
      Serial.print(m_W1(i,ii), precision);
      Serial.print('\t');
    }
  }
  Serial.println();

  Serial.println("W2---------------------------------");
  for(std::uint16_t i=0; i<m_h2_size; ++i){
    Serial.println();
    for(std::uint16_t ii=0; ii<m_h1_size; ++ii){
      Serial.print(m_W2(i,ii), precision);
      Serial.print('\t');
    }
  }

  Serial.println("W3---------------------------------");
  for(std::uint16_t i=0; i<m_out_size; ++i){
    Serial.println();
    for(std::uint16_t ii=0; ii<m_h2_size; ++ii){
      Serial.print(m_W3(i,ii), precision);
      Serial.print('\t');
    }
  }
}

// Save --------------------------------------------------------------------------------------------
void NeuralNetwork::SaveNetworkSD(const char * folder, const int iter, const char * xFile){

  char file[100];
  strcpy(file, xFile);

  char str_iter[5];
  sprintf(str_iter, "%d", iter);
  char xPath[100];

  strcpy(xPath, folder);
  strcat(xPath, file);
  strcat(xPath, str_iter);
  strcat(xPath, ".json");
    
 
    nlohmann::json jn;
    jn = nlohmann::json::parse(R"([])");

    // Save network weights and biasies
    for(uint16_t i=0; i < m_W1.rows(); i++) { jn[0]["W1"][i] = m_W1.row(i); }
    for(uint16_t i=0; i < m_W2.rows(); i++) { jn[0]["W2"][i] = m_W2.row(i); }
    for(uint16_t i=0; i < m_W3.rows(); i++) { jn[0]["W3"][i] = m_W3.row(i); }
    
    jn[0]["B1"] = m_B1;
    jn[0]["B2"] = m_B2;
    jn[0]["B3"] = m_B3;

    std::string serializedObject = jn.dump(4);
    writeFile(SD, xPath, serializedObject.c_str()); 
}

// Load --------------------------------------------------------------------------------------------
void NeuralNetwork::LoadNetworkSD(const char * folder, const int iter, const char * xFile){
 
  char file[100];
  strcpy(file, xFile);
  char str_iter[5];
  sprintf(str_iter, "%d", iter);
  char xPath[100];
  strcpy(xPath, folder);
  strcat(xPath, file);
  strcat(xPath, str_iter);
  strcat(xPath, ".json");
  Serial.print("Attempting to load");
  Serial.print('\t');
  Serial.println(xPath);
  auto j = readJsonFile(SD, xPath);

  m_W1 = Eigen::MatrixXf(int(j[0]["W1"].size()), int(j[0]["W1"][0].size()));
  m_W2 = Eigen::MatrixXf(int(j[0]["W2"].size()), int(j[0]["W2"][0].size()));
  m_W3 = Eigen::MatrixXf(int(j[0]["W3"].size()), int(j[0]["W3"][0].size()));

  m_B1 = Eigen::VectorXf(int(j[0]["B1"].size()));
  m_B2 = Eigen::VectorXf(int(j[0]["B2"].size()));
  m_B3 = Eigen::VectorXf(int(j[0]["B3"].size()));

  for(uint16_t r=0; r < m_W1.rows(); r++){for(uint16_t c=0; c < m_W1.cols(); c++){m_W1(r, c) = j[0]["W1"][r][c];}}
  for(uint16_t r=0; r < m_W2.rows(); r++){for(uint16_t c=0; c < m_W2.cols(); c++){m_W2(r, c) = j[0]["W2"][r][c];}}
  for(uint16_t r=0; r < m_W3.rows(); r++){for(uint16_t c=0; c < m_W3.cols(); c++){m_W3(r, c) = j[0]["W3"][r][c];}}

  for(uint16_t r=0; r < m_B1.size(); r++){m_B1(r) = j[0]["B1"][r];}
  for(uint16_t r=0; r < m_B2.size(); r++){m_B2(r) = j[0]["B2"][r];}
  for(uint16_t r=0; r < m_B3.size(); r++){m_B3(r) = j[0]["B3"][r];}
  
  m_in_size = j[0]["W1"][0].size();
  m_out_size = j[0]["W3"].size();
  m_h1_size  = j[0]["W1"].size();
  m_h2_size  = j[0]["W2"].size();
}


// Probe ------------------------------------------------------------------------------------
void NeuralNetwork::ProbeNetwork(){
  const float minX = -9;
  const float maxX =  9;
  const float minY = -9;
  const float maxY =  9;
  const float rangeX =  maxX - minX;
  const float rangeY =  maxY - minY;
//  float yVal = minX;
//  float xVal = minY;
  
  for(int x=0; x < 7; x++){
    const float xVal = (float(x)/3.0f-1.0f);
    for(int y=0; y < 7; y++){
      const int id = x*7 + y;
      const float yVal = (float(y)/3.0f-1.0f);
      m_in[0] = yVal;
      if (m_in_size>1){m_in[1] = xVal;}
      ForwardPropagate();
      m_probe[id] = m_out[0];
    }
  }
}



// Inverse Weights ------------------------------------------------------------------------------------------
void NeuralNetwork::WeightInverse(){
  /*
     Inverses all weights
  */
    // Generatr Weights
    for(std::uint16_t i=0; i<m_W1.size(); ++i){m_W1(i) = (m_W1(i)*-1.0f);}
    for(std::uint16_t i=0; i<m_W2.size(); ++i){m_W2(i) = (m_W2(i)*-1.0f);}
    for(std::uint16_t i=0; i<m_W3.size(); ++i){m_W3(i) = (m_W3(i)*-1.0f);}

    for(std::uint16_t i=0; i<m_B1.size(); ++i){m_B1(i) = (m_B1(i)*-1.0f);}
    for(std::uint16_t i=0; i<m_B2.size(); ++i){m_B1(i) = (m_B2(i)*-1.0f);}
    for(std::uint16_t i=0; i<m_B3.size(); ++i){m_B1(i) = (m_B3(i)*-1.0f);}
}
