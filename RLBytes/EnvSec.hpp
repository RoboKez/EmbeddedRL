#ifndef SECENV_H
#define SECENV_H
#include "Env.hpp"
#include "NN.hpp"

class SecEnv:public CoreEnv
{
public:
  float prev_pos = 0;
  float m_kill_speed = 2.0;
  float m_steps_in_danger_zone_limit_sec = 5;
  float m_steps_in_danger_zone_sec;

  virtual unsigned long episodeReset(bool manual_reset);
  virtual void markovStep(Eigen::VectorXf agent_act, int markovTime);
  uint16_t m_sub_steps;
  SecEnv(std::string task_name, uint16_t ob_size, uint16_t act_size, uint16_t step_limit, std::string pri_policy_folder, uint16_t pri_iter, uint16_t n_sub_steps);
  ~SecEnv();
  virtual void getOb();
  void LoadSubPolicy(std::string pri_policy_folder, int iter);
  NeuralNetwork *priNet = nullptr;
};

#endif // PRIENV_H
