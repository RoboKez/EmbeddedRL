 #include <SimpleFOC.h>
#include <Wire.h>

BLDCMotor motor0 = BLDCMotor(11);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(5, 10, 6, 8);
MagneticSensorI2C sensor0 = MagneticSensorI2C(0x40, 14, 0xFE, 8);
InlineCurrentSense current_sense0 = InlineCurrentSense(0.01, 50.0, A0, A2);

BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(9, 3, 11, 12); // solder pin 7 on driver and wire it to 12 on driver not the stm32
MagneticSensorI2C sensor1 = MagneticSensorI2C(0x44, 14, 0xFE, 8);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, A1, A3);


#define I2C_ADDR  2 // address of this slave on wire1 MAKE SURE EACH DRIVER ADDRESS IS UNIQUE!!
TwoWire Wire1(PC9, PA8); // second i2c bus (slave to esp) its stm32 i2c 3 bus
float minMov = 0.;
bool reboot = false;
unsigned long command_time;
float smooth_vel0 = 0;
float smooth_vel1 = 0;
bool core_command_since_start = false;
const float smooth_vel_ration = 0.95;  // higher more historic readings between 0 and 1.0
const float v_cutoff = 25;  // max speed recorded

void setup() {

//  Wire.setClock(400000);

  driver0.voltage_power_supply = 12;
  driver0.init();
  motor0.linkDriver(&driver0);
  sensor0.init();
  motor0.linkSensor(&sensor0);
  motor0.voltage_sensor_align = 7;
  motor0.controller = MotionControlType::torque;
  motor0.torque_controller = TorqueControlType::voltage;
  motor0.controller = MotionControlType::torque;
  motor0.phase_resistance = 11.1; // 11.1 Ohms
  motor0.linkCurrentSense(&current_sense1);
  current_sense0.gain_b *=-1;
  current_sense0.skip_align = true;

  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  sensor1.init();
  motor1.linkSensor(&sensor1);
  motor1.voltage_sensor_align = 7;
  motor1.controller = MotionControlType::torque;
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor1.phase_resistance = 11.1; // 11.1 Ohms
  motor1.linkCurrentSense(&current_sense1);
  current_sense1.gain_b *=-1;
  current_sense1.skip_align = true;

  motor0.init();
  motor0.initFOC(0.05, CW);
  motor1.init();
  motor1.initFOC(1.3, CW);

//  motor0.init();
//  motor0.initFOC(3.6252, CW);
//  motor1.init();
//  motor1.initFOC(6.0055, CCW);

  Serial.begin(115200);
  _delay(10);

  // slave setup
  Wire.begin();
  Wire1.begin(I2C_ADDR);  // join i2c bus with address #4
  Wire1.onRequest(requestEvent); // request event
  Wire1.onReceive(receiveEvent); // register event
  Serial.println("driver slave ready");
  _delay(10);

  motor0.target = 0.0;
  motor1.target = 0.0;
  command_time = millis();
  
}


void loop() {
  /* Send action to motors, if no action recieved in 700ms motors stop (safety feature)*/
//  if(((millis()-command_time) > 700) and core_command_since_start){
//    Serial.println("waiting too long");
//    NVIC_SystemReset();
//  }

  smooth_vel0 = smooth_vel0 * smooth_vel_ration + motor0.shaft_velocity* (1.0-smooth_vel_ration);
  smooth_vel1 = smooth_vel1 * smooth_vel_ration + motor1.shaft_velocity* (1.0-smooth_vel_ration);
    
  // main FOC algorithm function
  motor0.loopFOC();
  motor1.loopFOC();

  // Motion control function
  motor0.move();
  motor1.move(); 
}



void receiveEvent(int howMany)
{
  
  while(1 < Wire1.available()) // loop through all but the last
  {
     motor0.target = (act_byte_to_float(Wire1.read())*2.0);     // receive byte as a character
     motor1.target = (act_byte_to_float(Wire1.read())*2.0);     // receive byte as a character
  }
  smooth_vel0 = motor0.shaft_velocity;
  smooth_vel1 = motor1.shaft_velocity;
  core_command_since_start = true;
}

void requestEvent()
{
  command_time = millis();
  byte p0 = byte_position(motor0.shaft_angle);
  byte p1 = byte_position(motor1.shaft_angle);
  byte v0 = float_to_byte(smooth_vel0, v_cutoff);
  byte v1 = float_to_byte(smooth_vel1, v_cutoff);
//  Serial.println("--------------------------");
//  Serial.print(v0);
//  Serial.print('\t');
//  Serial.println(v1);
  Wire1.write(v0);
  Wire1.write(v1);
}

uint8_t byte_position(float cont_angle){
  byte senB;
  int senI;
  float senF;
  senI = int(cont_angle*10000);
  senI = senI % 62831;
  if (senI < 0) {
    senI = 62831 + senI;
    }
  senF = (float)senI/246.396078431;
  senI = round(senF);
  return byte(senI);
}


float act_byte_to_float(byte a_byte) {
  float a_float = (a_byte - 127) * 0.003;
  if (a_byte < 127) {
    a_float -= minMov;
  } else if (a_byte > 127) {
    a_float += minMov;
  } else { // byte_a = 127
    a_float = 0.0; //0.01; // as sending 0.0 causes issues with the old 6050 IMU for some reason
  }
  return a_float;
}

/* Convert limited float into bytes for cutoff range */
uint8_t float_to_byte(float float_, float cutoff) {
  float_ = constrain(float_, -cutoff, cutoff);
  return byte(127.5f + float_*127.0f/cutoff);
}
