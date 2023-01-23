#include "Env.hpp"


// CoreEnv ===========================================================================================
CoreEnv::CoreEnv(std::string task_name, uint16_t ob_size, uint16_t act_size, uint16_t step_limit, std::string sub_task_name)
:m_name(task_name), m_ob_size(ob_size), m_act_size(act_size), m_step_limit(step_limit)
{
    Serial.println(task_name.c_str());
    Serial.println(sub_task_name.c_str());

    if(task_name == "P_Primary"){
      m_pri_ob_mode = 0;
    } else if (task_name == "PI_Primary"){
      m_pri_ob_mode = 1;
    } else if (task_name == "PID_Primary"){
      m_pri_ob_mode = 2;
    } else if (task_name == "PVPV_Primary"){
      m_pri_ob_mode = 3;
    } else if (task_name == "PD_Primary"){
      m_pri_ob_mode = 4;
    } else if (task_name == "Classic_Primary"){
      m_pri_ob_mode = 5;
    } else if (task_name == "ClassicA_Primary"){
      m_pri_ob_mode = 6;
      
    } else if (task_name == "P_Secondary"){
      m_sec_ob_mode = 0;
    } else if (task_name == "PI_Secondary"){
      m_sec_ob_mode = 1;
    } else if (task_name == "PID_Secondary"){
      m_sec_ob_mode = 2;
    } else if (task_name == "PD_Secondary"){
      m_sec_ob_mode = 4;
    } else {
      while(1){Serial.println("RL Error: invalid ob mode, check task name or add to core Env constructor if new task");}
    }

    m_pri_ob_size = m_ob_size;

    if (sub_task_name=="/P_Sub"){
      m_pri_ob_mode = 0;
      m_pri_ob_size = 1;
    } else if (sub_task_name=="/PI_Sub"){
      m_pri_ob_mode = 1;
      m_pri_ob_size = 2;
    } else if (sub_task_name=="/PID_Sub"){
      m_pri_ob_mode = 2;
      m_pri_ob_size = 3;
    } else if (sub_task_name == "/PD_Sub"){
      m_pri_ob_mode = 4;
      m_pri_ob_size = 2;
    } else if (sub_task_name==""){
      Serial.println("Non hieracical");
    } else {
      while(1){Serial.println("RL Error: invalid hieracical ob mode, check task name or add to core Env constructor if new task"); Serial.println(sub_task_name.c_str()); delay(1000);}
    }
    Serial.print("ob mode"); Serial.println(m_pri_ob_mode);
  
  m_ob = Eigen::VectorXf(ob_size);
  m_act = Eigen::VectorXf(act_size);
  priPID = StandardPID(1.0);  // Note we don't have any gains as the NN replaces them, but we do have an intergral limit
  secPID = StandardPID(1.0);  // Note we don't have any gains as the NN replaces them, but we do have an intergral limit
  m_quit = false;
  Serial.println("Core Env init");
  strcat(m_folder, m_name.c_str());
  removeDir(SD, m_folder);  // if exists
}

CoreEnv::~CoreEnv()
{
    Serial.println("Core destroyed");
}



// Step ----------------------------------------------------------------------------------------------
void CoreEnv::markovStep(Eigen::VectorXf agent_act, int markovTime)
{


  if(smoothed_action){
    int sub_steps = markovTime;
    float dif_act = (agent_act[0] - m_prev_act);
    for(int i=0; i<sub_steps; i++){
      float smoothed_act = m_prev_act + dif_act*float(i)/float(sub_steps);
      sendTorque(smoothed_act+turn, smoothed_act-turn);
      delay(1);
     }
  } else {
    if(agent_act.size() == 1){sendTorque(agent_act[0]+turn, agent_act[0]-turn);} 
    else if (agent_act.size() == 2) {sendTorque(agent_act[0], agent_act[1]);} 
    else {Serial.print("RL Error: agent act max is 2 actions, you have "); Serial.println(agent_act.size());
    }
  }

   m_prev_act = agent_act[0];
  delay(markovTime);
  m_step +=1;
  getOb(); // Get next observation, rewards and done
}

// Ob mode --------------------------------------------------------------------------------------------
VectorXf CoreEnv::priOb(){
  updatePitch();
  updateWheels();
  priPID.update(m_pri_true, m_pri_set_point);
  secPID.update(m_sec_true, m_sec_set_point);
  VectorXf tmp_ob = VectorXf(m_pri_ob_size);
  
  // P
  tmp_ob[0] = priPID.P;  // this is just m_pri_true - m_pri_set_point

  // PI
  if (m_pri_ob_mode == 1){
    tmp_ob[1] = priPID.I;

  // PID
  } else if (m_pri_ob_mode == 2){
    tmp_ob[1] = m_gyro*0.01; //priPID.D*0.1;
    tmp_ob[2] = priPID.I;

 // PD
  } else if (m_pri_ob_mode == 4){
    tmp_ob[1] = m_gyro*0.01;// priPID.D*0.1;  // normalise input dependant of freqency  // m_gyro*0.01; //priPID.D;
//    tmp_ob[1] = priPID.D*0.1;
  

  // PVPV
  } else if (m_pri_ob_mode == 3){
    tmp_ob[1] = m_gyro*0.01;// m_pitch_vel;
    tmp_ob[2] = m_cur_cart_pos;  //add in or minus right
    tmp_ob[3] = m_cart_vel;

  // Classic
  } else if (m_pri_ob_mode == 5){
    tmp_ob[1] = m_gyro*0.01;// m_pitch_vel;
    tmp_ob[2] = secPID.P; //m_cur_cart_pos/6.284; //1=full rotation //add in or minus right
    tmp_ob[3] = m_cart_vel;

  // Classic and Previous Action
  } else if (m_pri_ob_mode == 6){
    tmp_ob[1] = m_gyro*0.01;// m_pitch_vel;
    tmp_ob[2] = secPID.P; //m_cur_cart_pos/6.284; //1=full rotation //add in or minus right
    tmp_ob[3] = m_cart_vel;
    tmp_ob[4] = m_prev_act; // assumes 1d action space
  }
  
  return tmp_ob;
}

VectorXf CoreEnv::secOb(){
//  updateLidar();
//  m_sec_true = 1.0f - m_lidarDis/127.5f;  // scale between -1 and 1
//  m_sec_true = 1.0f - m_lidarAng/11.5;  // scale between -1 and 1
  secPID.update(m_sec_true, m_sec_set_point);
  VectorXf tmp_ob = VectorXf(m_ob_size);
  
  // P
  tmp_ob[0] = secPID.P;  // this is just m_pri_true - m_pri_set_point

  // PD
  if (m_pri_ob_mode == 4){
    tmp_ob[1] = priPID.D;
  }
  
  return tmp_ob;
}



// Pitch ----------------------------------------------------------------------------------------------
void CoreEnv::updatePitch()
{
  const float prev_pitch = m_pitch;
  const float prev_vel = m_pitch_vel;
  
  const float pitch_range = 30.0f;      // -30 to 30 degrees used for normalising the network inputs 
  const float sensor_correction = 3.0f; // correction angle degrees if sensor is out by a constant amount
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  m_pitch = (euler.z() - sensor_correction);
  m_gyro = gyro.x();
   
  m_pitch_vel = m_pitch - prev_pitch;
  m_pitch_acc = m_pitch_vel - prev_vel;

//  Serial.print("Pitch pos\t"); Serial.println(m_pitch);
//  Serial.print("Pitch vel\t"); Serial.println(m_pitch_vel);
//  Serial.print("Pitch acc\t"); Serial.println(m_pitch_acc);

  m_pri_true = m_pitch/pitch_range;

  
  setPixel(-1, m_pri_true);
}

// Wheels ---------------------------------------------------------------------------------------------
void CoreEnv::updateWheels()
{

//  encoderL.update();
//  encoderR.update();
  float prev_vL = m_vL;
  float prev_vR = m_vR;

//  m_pL = encoderL.getAngle() - m_pL_Start;
//  m_pR = encoderR.getAngle() - m_pR_Start;
posL = countL/500.0f*6.28318530718; 
  posR = countR/500.0f*6.28318530718;
 m_pL = posL - m_pL_Start;
m_pR = posR - m_pR_Start;
//  Serial.print(m_pL);
//  Serial.print('\t');
//  Serial.print(m_pR);
//  Serial.print('\t');
//  Serial.println(m_pL - m_pR);
  
//  m_vL = encoderL.getVelocity();
//  m_vR = encoderR.getVelocity();
//  m_vL = posL;
//  m_vR = posR;
  

//  m_sec_true = (m_vL - m_vR) * 0.05f;

  float tmp_prev = m_prev_cart_pos;
  m_prev_cart_pos = m_cur_cart_pos;
//  m_cur_cart_pos = ((m_pL - m_pR) * 0.5f +  0.0174533*env->m_pitch) - m_ang_Start; //*0.5 + 0.5*tmp_prev;
m_cur_cart_pos = ((m_pL + m_pR) * 0.5f +  0.0174533*env->m_pitch) - m_ang_Start; //*0.5 + 0.5*tmp_prev;
//  Serial.println(m_cur_cart_pos);
  m_cart_vel = (m_prev_cart_pos - m_cur_cart_pos)*10.0f; 
  m_sec_true = -m_cur_cart_pos/6.284; 
//   Serial.print("Wheel pos\t"); Serial.print(m_pL); Serial.print('\t'); Serial.println(m_pR);
//   Serial.print("Wheel vel\t"); Serial.print(m_vL); Serial.print('\t'); Serial.println(m_vR);
//   Serial.print("Wheel acc\t"); Serial.print(m_aL); Serial.print('\t'); Serial.println(m_aR);
   
  
//  sensorL.update();
//  sensorR.update();
//  m_vL = m_vL * m_smooth_vel_ratio + sensorL.getVelocity() * (1.0-m_smooth_vel_ratio);
//  m_vR = m_vR * m_smooth_vel_ratio + sensorR.getVelocity() * (1.0-m_smooth_vel_ratio);
}

void CoreEnv::zeroWheelPos()
{
//  encoderL.update();
//  encoderR.update();
//  m_pL_Start = encoderL.getAngle();
//  m_pR_Start = encoderR.getAngle();
  posL = countL/500.0f*6.28318530718; 
  posR = countR/500.0f*6.28318530718;

  m_pL_Start = posL;
  m_pR_Start = posR;
  
  m_vL = 0;
  m_vR = 0;
  m_cur_cart_pos = 0;
  Serial.println("Wheel positions zeroed");
}

// Lidar ----------------------------------------------------------------------------------------------
void CoreEnv::updateLidar()
{
  const byte bytes_requested = 2;
  byte pos_[2] = {127, 127};
  Wire.requestFrom(byte(I2C_ADDR_LIDAR), bytes_requested);  // Request 2 bytes, change if state size changes!!!
  byte bytes_recieved;
  bytes_recieved = Wire.readBytes(pos_, bytes_requested);
  m_lidarDis = float(pos_[1]);
  m_lidarAng = float(pos_[0]);
}

// I2C Scan ---------------------------------------------------------------------------------------------
bool i2c_devices_found()
{
  bool imu_found = false;
  bool driver1_found = false;
  bool lidar_found = false;
  byte error;

  Serial.println("Scanning...");
    // IMU
    Wire.beginTransmission(40);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(40, HEX);
      imu_found = true;  // Note 0x28 from hex to int is 40!!
    }

    delay(10);

    // Motor Driver
    Wire.beginTransmission(I2C_ADDR_BRUSHLESS);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(I2C_ADDR_BRUSHLESS, HEX);
      driver1_found = true;
    }

   delay(10);

   // Lidar
    Wire.beginTransmission(I2C_ADDR_LIDAR);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(I2C_ADDR_LIDAR, HEX);
      lidar_found = true;
    }

   delay(10);

       
  if (imu_found and driver1_found and lidar_found ) {
    Serial.print("all i2c");
    digitalWrite(13, HIGH);
    return true;
  } else {
    Serial.println("still waiting on following i2c devices...");
    if (not imu_found) Serial.println("imu");
    if (not driver1_found) Serial.println("driver1");
    if (not lidar_found) Serial.println("lidar");
    return false;
    }
}


// sendTorque ===========================================================================================
void sendTorque(float left, float right)
{
  left = max(min(left, 0.8f), -0.8f);
  right = max(min(right, 0.8f), -0.8f);

  byte left_byte  = byte( left * 127.5f + 127.5f);
  byte right_byte  = byte( right * 127.5f + 127.5f);

  Wire.beginTransmission(I2C_ADDR_BRUSHLESS);     // transmit to device
  Wire.write(left_byte);                      // sends one byte
  Wire.write(right_byte);                     // sends one byte
  Wire.endTransmission();                // stop transmitting 
}

// setDisplays ==========================================================================================
void setEyes(int eyeMode){
  if (eyeMode == 0)
  {
    // sleep
    clearAllPixels(false);
    for(int i=0; i<8; i++){
      setPixel(i, 0, false);
      setPixel(i+64, 0, false);
    }
    matricies_strip.show();
    
  } else if (eyeMode == 1){
    
    clearAllPixels(false);
    setPixel(27, 1, false);
    setPixel(28, 1, false);
    setPixel(35, 1, false);
    setPixel(36, 1, false);

    setPixel(27+64, -1, false);
    setPixel(28+64, -1, false);
    setPixel(35+64, -1, false);
    setPixel(36+64, -1, false);
    matricies_strip.show();
    
    
  } else if (eyeMode == 2){
    
    clearAllPixels(false);
    setPixel(27, 1, false);
    setPixel(28, 1, false);
    setPixel(35, 1, false);
    setPixel(36, 1, false);

    setPixel(27+64, 1, false);
    setPixel(28+64, 1, false);
    setPixel(35+64, 1, false);
    setPixel(36+64, 1, false);
    matricies_strip.show();

  } else if (eyeMode == 3){
    
    clearAllPixels(false);
    setPixel(27, -1, false);
    setPixel(28, -1, false);
    setPixel(35, -1, false);
    setPixel(36, -1, false);

    setPixel(27+64, -1, false);
    setPixel(28+64, -1, false);
    setPixel(35+64, -1, false);
    setPixel(36+64, -1, false);
    matricies_strip.show();

  } else if (eyeMode == 4){
    
    clearAllPixels(false);
    setPixel(27, 0, false);
    setPixel(28, 0, false);
    setPixel(35, 0, false);
    setPixel(36, 0, false);

    setPixel(27+64, 0, false);
    setPixel(28+64, 0, false);
    setPixel(35+64, 0, false);
    setPixel(36+64, 0, false);
    matricies_strip.show();
    
    
  }  else if (eyeMode == 5){
    
    clearAllPixels(false);
    setPixel(0, 0, false);
    setPixel(1, 1, false);
    setPixel(2, -1, false);
    setPixel(3, 0, false);
    matricies_strip.show();
    
    
  }
  else {
    clearAllPixels(false);
    for(int i=0; i<7; i++){
      setPixel(i, 0, false);
    }
    matricies_strip.show();
  }
}

// set onboard dotstar pixel ==========================================================================
void setPixel(const int id, const float Val, const bool showNow, float brightness)
/*
 * Sets pixel <id> with rgb value where -1=red and 1=blue at bightness where 1.0 is 100%
 * Note -1 is the onboard pixel which is on a different dotstar strip
 */
{
    brightness = max(min(brightness, 1.0f), 0.0f);
    const float minVal = -1.0;
    const float maxVal = 1.0;
    const float reading = max(min(Val, maxVal), minVal);
    const float ratio = 2.0f * (reading - minVal) / ((maxVal - minVal)+0.000001);
    const byte r_ = (max(0.0f, 255.0f*(1.0f - ratio)));
    const byte b_ = (max(0.0f, 255.0f*(ratio - 1.0f)));
    const byte g_ = 255 - b_ - r_; 
    
    
    if(id == -1)
    {
      onboard_strip.setPixelColor(0, g_*brightness, b_*brightness, r_*brightness);
      onboard_strip.show();
    } else {
      matricies_strip.setPixelColor(id, g_*brightness, b_*brightness, r_*brightness);
      if(showNow) {matricies_strip.show();}   
    }        
}

// set onboard dotstar pixel ==========================================================================
void clearAllPixels(bool showOff)
/*
 * Clears all matrices values
 * showOff if true sends command to dotstar to turn of pixels
 */
{
      for(int i=0; i<128; i++){matricies_strip.setPixelColor(i, 0,0,0); }
      matricies_strip.show();
}


// set onboard dotstar pixel ==========================================================================
void setGrid(int x, int y, float Val, bool polNet)
/*
 * Sets pixel at grid x,y but holds of on showing
 */
{
  int id = x+ y*8;
  if(polNet){id += 64;} 
  Serial.println(id);
  setPixel(id, Val, true);
}

// set onboard dotstar pixel ==========================================================================
/*
 * Set pixels to -1 to 1 probe values (except row and columb 8
*/
void setPixelNN(VectorXf nnP, VectorXf nnV, bool normaliseV){
  for(int x=0; x<7; x++){
    for(int y=0; y<7; y++){  
       const int grid_id = x*8 + y;
       const int nn_id = x*7 + y;
       setPixel(grid_id, nnP[nn_id], false);
    }
  }


  float maxV = nnV.maxCoeff();
  float minV = nnV.minCoeff();
  float rangeV = maxV - minV;

  for(int x=0; x<7; x++){
    for(int y=0; y<7; y++){  
       const int grid_id = x*8 + y + 64;
       const int nn_id = x*7 + y;
       const float v = (((nnV[nn_id]-minV)/rangeV)-0.5f)*2.0f;
       if (normaliseV){
        setPixel(grid_id, v, false);
       } else {
        setPixel(grid_id, nnV[nn_id], false);
       } 
    }
  }
//  setPixel(7, -1, false);
//  setPixel(15, 0, false);
//  setPixel(23, 1, false);
  matricies_strip.show();
  
}

uint16_t CoreEnv::getObSize(){return m_ob_size;}
uint16_t CoreEnv::getActSize(){return m_act_size;}
float CoreEnv::getReturn(){return m_episode_return;}
