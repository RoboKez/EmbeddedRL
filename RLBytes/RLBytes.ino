#include "RLBytes.hpp"

void ErrorMessageLoop(const char * message){
  while(1){
    Serial.print("RL Error: ");
    Serial.println(message);
    delay(1000);
  }
}

void robotBegin(){
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  matricies_strip.begin(); // Initialize pins for output
  onboard_strip.begin(); // Initialize pins for output
  matricies_strip.setBrightness(20); // 10 min 100 max
  onboard_strip.setBrightness(20); // 10 min 100 max
  matricies_strip.show();  // Turn all LEDs off ASAP
  onboard_strip.show();  // Turn all LEDs off ASAP
  encoderL.init();
  encoderL.enableInterrupts(doL1, doL2);
  encoderR.init();
  encoderR.enableInterrupts(doR1, doR2);
  delay(1000);
  i2c_devices_found();
  if(!bno.begin()){ErrorMessageLoop("No BNO055 detected");}
  delay(100);
  bno.setExtCrystalUse(true);
  delay(100);

  if(!SD.begin()){
    ErrorMessageLoop("Card mount failed... did you put the card in?");
  } else {
    digitalWrite(LED_BUILTIN, HIGH); 
    Serial.println("SD successfully initialised");
  }
  delay(1000);
  setEyes(1);
}


void newExperiment(std::string env_name,  
                  int step_limit, 
                  int h1, 
                  int h2, 
                  float lr, 
                  int seed, 
                  int batch_size, 
                  int mini_batch_size, 
                  bool adam,
                  std::string sub_env_folder, 
                  int sub_env_iter, 
                  int sub_repeats){
                    
  // 0:P_Primary, 1:PI_Primary,
  
    
 
  delete agent;
  delete env;
  iteration = -1;

  if(minibatch_size > batch_size){batch_size = minibatch_size;}
  

  if (env_name=="P_Primary"){
    env = new PriEnv(env_name, 1, 1, step_limit);
    
  }else if (env_name=="PI_Primary") {
    env = new PriEnv(env_name, 2, 1, step_limit);

  }else if (env_name=="PD_Primary") {
    env = new PriEnv(env_name, 2, 1, step_limit);

  }else if (env_name=="PID_Primary") {
    env = new PriEnv(env_name, 3, 1, step_limit);
    
  } else if (env_name=="PVV_Primary") { //pitch pos, pitch vel, wheel vel
    env = new PriEnv(env_name, 3, 1, step_limit);
  } else if (env_name=="Classic_Primary") { //pitch pos, pitch vel, cart pos, cart vel
    env = new PriEnv(env_name, 4, 1, step_limit);
  } else if (env_name=="P_Secondary") {
    env = new SecEnv(env_name, 1, 1, step_limit, sub_env_folder, sub_env_iter, sub_repeats);
  } else {
    ErrorMessageLoop("Invalid task name see RLBytes.cpp");
  };

  env->m_steps_in_danger_zone_limit = steps_in_danger_zone_limit;
  env->m_kill_angle = kill_angle;
  
  agent = new Agent(
    env->getObSize(),
    env->getActSize(),
    h1,                    
    h2,                    
    lr,                    
    seed,      
    batch_size,   
    mini_batch_size,     
    step_limit,
    false, 
    adam 
  );

  agent->m_standard_deviation = standard_deviation;
  agent->m_min_standard_deviation = min_standard_deviation;
  agent->m_anneal_rate = anneal_rate;
    
  createDir(SD, env->m_folder);
}

void loadExperiment(const char * folder, int iter){
  // Ensure SD set up prior to running 
  char file[100] = "/info_";
  char str_iter[5];
  sprintf(str_iter, "%d", iter);
  char xPath[100];
  strcpy(xPath, folder);
  strcat(xPath, file);
  strcat(xPath, str_iter);
  strcat(xPath, ".json");
  Serial.print("load info at "); Serial.println(xPath);
  auto j = readJsonFile(SD, xPath);
  
  n_epochs = j[0]["epochs"];
  markov_time = j[0]["markov_time"];
  std::string tmpname = j[0]["env"].get<std::string>();
  Serial.println(tmpname.c_str());
  const char * a = tmpname.c_str();
  newExperiment(a, j[0]["step_limit"], j[0]["h1_size"], j[0]["h2_size"], j[0]["lr"], j[0]["seed"], j[0]["desired_batch_size"], j[0]["minibatch_size"], j[0]["adam"]);
  agent->LoadNetworkSD(folder, iter);
}

void saveInfo(){
  
  char file[100] = "/info_";
  char str_iter[5];
  sprintf(str_iter, "%d", iteration);
  char xPath[100];
  strcpy(xPath, env->m_folder);
  strcat(xPath, file);
  strcat(xPath, str_iter);
  strcat(xPath, ".json");
  
  nlohmann::json j;
  j = nlohmann::json::parse(R"([])");

  j[0]["epochs"] = n_epochs;
  j[0]["adam"] = agent->m_adam;
  j[0]["step_limit"] = env->m_step_limit;
  j[0]["lr"] = agent->m_lr;
  j[0]["seed"] = agent->m_seed;
  j[0]["desired_batch_size"] = agent->m_desired_batch_size;
  j[0]["minibatch_size"] = agent->m_minibatch_size;
  j[0]["deterministic_return"] = batch_cum_det/eps_det;
  j[0]["socastic_return"] = batch_cum/eps;
  j[0]["ID"] = iteration;
  j[0]["env"] = env->m_name;
  j[0]["total_training_steps"] = total_training_steps;
  j[0]["in_size"] = agent->m_in_size;
  j[0]["act_size"] = agent->m_act_size;
  j[0]["h1_size"] = agent->m_h1_size;
  j[0]["h2_size"] = agent->m_h2_size;
  j[0]["value_probe"] = agent->vNet.m_probe;
  j[0]["policy_probe"] = agent->pNet.m_probe;
  j[0]["markov_time"] = markov_time;
  j[0]["kill_angle"] = kill_angle;
  j[0]["steps_in_danger_zone_limit"] = standard_deviation;
  j[0]["min_standard_deviation"] = min_standard_deviation;
  j[0]["anneal_rate"] = anneal_rate;

  std::string serializedObject = j.dump(4);
  writeFile(SD, xPath, serializedObject.c_str()); 
}

void iterReset(){
  iteration += 1;
  clearAllPixels(false);
  setPixelNN(agent->pNet.m_probe, agent->vNet.m_probe);
  Serial.print("Iteration: "); Serial.println(iteration); 
  iter_timestamp      = millis();
  batch_cum           = 0;
  batch_cum_det       = 0;
  eps                 = 0;
  eps_det             = 0;
  soc_manual_time     = 0;
  soc_wait_time       = 0;
  det_manual_time     = 0;
  det_wait_time       = 0;
  train_time          = 0;
  test_time           = 0;
  learn_time          = 0;
  iter_time           = 0;
}
