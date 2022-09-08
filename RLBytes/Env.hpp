#ifndef ENV_H
#define ENV_H
#include "pid.hpp"
#include "parameters.h"
/*
 *  `CoreEnv
 */

#include <ArduinoEigen.h>
#include <WiFi.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_DotStar.h>
#include <APA102.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <esp_now.h>

#define LED_BUILTIN         13
#define NUMPIXELS_ONBOARD   1 
#define DATAPIN_ONBOARD     40
#define CLOCKPIN_ONBOARD    45
#define NUMPIXELS_MATRICIES 128 
#define DATAPIN_MATRICIES   17
#define CLOCKPIN_MATRICIES  18
#define I2C_ADDR_BRUSHLESS  2
#define I2C_ADDR_LIDAR      0x4B
//#define I2C_ADDR_LEFT_MAG   0x60
//#define I2C_ADDR_RIGHT_MAG  0x4C
#define I2C_ADDR_POL        0x70
#define I2C_ADDR_VAL        0x72
//#define BNO055_SAMPLERATE_DELAY_MS (10)

#define ENCL_A 11
#define ENCL_B 10
#define ENCR_A 38
#define ENCR_B 33


float m_prev_act = 0;
Encoder encoderL = Encoder(ENCL_A, ENCL_B, 500);
void doL1() {  encoderL.handleA(); }
void doL2() {  encoderL.handleB(); }

Encoder encoderR = Encoder(ENCR_A, ENCR_B, 500);
void doR1() {  encoderR.handleA(); }
void doR2() {  encoderR.handleB(); }

Adafruit_DotStar onboard_strip(NUMPIXELS_ONBOARD, DATAPIN_ONBOARD, CLOCKPIN_ONBOARD, DOTSTAR_BRG);
Adafruit_DotStar matricies_strip(NUMPIXELS_MATRICIES, DATAPIN_MATRICIES, CLOCKPIN_MATRICIES, DOTSTAR_BRG);

//MagneticSensorI2C sensorL = MagneticSensorI2C(I2C_ADDR_LEFT_MAG, 14, 0xFE, 8);
//MagneticSensorI2C sensorR = MagneticSensorI2C(I2C_ADDR_RIGHT_MAG, 14, 0xFE, 8); 
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

using Eigen::MatrixXf;
using Eigen::VectorXf;
const float smooth_ratio = 0.9;  // higher more historic readings between 0 and 1.0

void setPixel(const int id, const float Val, const bool showNow = true, float brightness = 0.4);
void clearAllPixels(bool showOff = true);
void setGrid(int x, int y, float Val, bool polNet);
void setEyes(int eyeMode);
void setPixelNN(VectorXf nnP, VectorXf nnV, bool normaliseV=true);
bool i2c_devices_found();

class CoreEnv
{
public:
  virtual void getOb() = 0;
  virtual unsigned long episodeReset(bool manual_reset) = 0;
  VectorXf priOb();
  VectorXf secOb();
  virtual ~CoreEnv();
  uint16_t getObSize();
  uint16_t getActSize();
  float getReturn();
  
  virtual void markovStep(Eigen::VectorXf agent_act, int markovTime);
  void updateWheels();
  void zeroWheelPos();
  void updateLidar();
  void updatePitch();
  
  int m_act_size;
  int m_ob_size;
  VectorXf m_ob;
  VectorXf m_act;
  float m_reward;
  uint8_t m_done;
  int m_step;
  std::string m_name; 
  int m_step_limit;
  bool m_quit;
  char m_folder[100] = "/";
  float m_episode_return;
  int m_pri_ob_mode;
  int m_sec_ob_mode;
  
  float m_pri_true = 0;
  float m_sec_true = 0;
  float m_pri_set_point = 0;
  float m_sec_set_point = 0;
  float m_kill_angle = kill_angle;
  float m_steps_in_danger_zone;
  float m_steps_in_danger_zone_limit;
  float m_smooth_vel_ratio = 0.95;  // 0 is no smoothing
  
  float m_pitch = 0;
  float m_gyro = 0;
  float m_pitch_vel = 0;
  float m_pitch_acc = 0;
  float m_lidarDis = 0;
  float m_lidarAng = 0; 

  float m_pL_Start = 0;
  float m_pR_Start = 0;
  float m_pL = 0;
  float m_pR = 0;
  float m_vL = 0;
  float m_vR = 0;
  float m_aL = 0;
  float m_aR = 0;
  
  float m_pitch_inv = 1.0f; 


protected:
  CoreEnv(std::string task_name, uint16_t ob_size, uint16_t act_size, uint16_t step_limit, std::string sub_task_name="");
  StandardPID priPID = NULL;
  StandardPID secPID = NULL;
};

#endif // ENV_H
