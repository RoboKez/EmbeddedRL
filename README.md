# EmbeddedRL
------------
Welcome to EmbeddedRL. This is the real world version of AppRL - a lightweight c++ libary for actor-critic based reinforcement learning on embedded systems. <br/>

![Alt text](/Images/e1.png?raw=true "Carrie")

Reference
----------
RLBytes does not use gym, stable baselines or any other existing RL libaries, instead it uses my  implimentation of Proximal Policy Optimisation (PPO) 
If you are interested I have a version of RLBytes which I made into an app for the Apple M1. It uses a c_bridge and the MuJoCo physics engine instead of the real world. 
<br/>

PPO:          Undelying algorithm we impliment https://openai.com/blog/openai-baselines-ppo/<br/>
NlohmannJon:  Saving and loading data https://github.com/nlohmann/json<br/>
SimpleFOC:    Brushless motor torque control and wheel velocity sensing: https://github.com/nlohmann/json<br/>
Eigen:        Matrix multiplication<br/>
Adafruit:     BNO055 as the IMU sensor<br/>
Adafruit:     Dotstar for visualising the probed value and policy networks<br/>
