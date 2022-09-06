# EmbeddedRL
------------
Welcome!
Hierarchical, Curriculum, Model and Vanilla Reinforcement learning based on Proximal Policy Optimisation for embedded systems such as the ESP32 
This if for P.I.P.E.R Policy Iteraction, Policy Evaluvation Robot a segway style robot that learning to balance and follow a moving target though hieracical learning
Note Piper uses stocastic policy gradient methods not policy iteration and evaluation, but its made from pipes and the name sounds cool so we could it that as a shout out to earlier work.

Reference
----------
We do not use gym, stable baselines or any other existing RL libaries, instead we wrote our own which we called RLBytes which is included in this repo. 
If you are interested we have a version of RLBytes which we made into an app for the Apple M1. It uses a c_bridge and MuJoCoTM instead of the real world. 

NlohmannJon:  Saving and loading data https://github.com/nlohmann/json
SimpleFOC:    Brushless motor torque control and wheel velocity sensing: https://github.com/nlohmann/json
Eigen:        Matrix multiplication
Adafruit:     BNO055 as the IMU sensor
Adafruit:     Dotstar for visualising the probed value and policy networks

CCheers,
KKez
