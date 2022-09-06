# EmbeddedRL
------------
Welcome to EmbeddedRL. This is a version of our RLBytes c++ libary for the embedded systems. <br/>

Enjoy Hierarchical, Curriculum, Model and Vanilla Reinforcement learning based on Proximal Policy Optimisation (PPO)
This if for P.I.P.E.R Policy Iteraction, Policy Evaluvation Robot a segway style robot that learning to balance and follow a moving target though hieracical learning
Note Piper uses stocastic policy gradient methods not true policy iteration and evaluation, but its made from pipes and the name sounds cool so we named it  as a shout out to earlier work.

Reference
----------
We do not use gym, stable baselines or any other existing RL libaries, instead we wrote our own which we called RLBytes which is included in this repo. 
If you are interested we have a version of RLBytes which we made into an app for the Apple M1. It uses a c_bridge and MuJoCoTM instead of the real world. 
<br/>

PPO:          Undelying algorithm we impliment https://openai.com/blog/openai-baselines-ppo/<br/>
NlohmannJon:  Saving and loading data https://github.com/nlohmann/json<br/>
SimpleFOC:    Brushless motor torque control and wheel velocity sensing: https://github.com/nlohmann/json<br/>
Eigen:        Matrix multiplication<br/>
Adafruit:     BNO055 as the IMU sensor<br/>
Adafruit:     Dotstar for visualising the probed value and policy networks<br/>
