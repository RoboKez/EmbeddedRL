/**
 * Simple example intended to help users find the zero offset and natural direction of the sensor. 
 * 
 * These values can further be used to avoid motor and sensor alignment procedure. 
 * 
 * motor.initFOC(zero_offset, sensor_direction);
 * 
 * This will only work for abosolute value sensors - magnetic sensors. 
 * Bypassing the alignment procedure is not possible for the encoders and for the current implementation of the Hall sensors. 
 * library version 1.4.2.
 * 
 * 5.1664 3.5381  5.5127
Direction::CW Direction::CCW  Direction::CW



Sensor zero offset is:
2.2170
Sensor natural direction is: 
Direction::CW
Mid Motor 1
Sensor zero offset is:
5.7321
Sensor natural direction is: 
Direction::CW


2.4827
Sensor natural direction is: 
Direction::CW
Mid Motor 1
Sensor zero offset is:
5.3942
Sensor natural direction is: 
Direction::CW


 */
#include <SimpleFOC.h>

//BLDCMotor motor2 = BLDCMotor(11);
//BLDCDriver3PWM driver2 = BLDCDriver3PWM(PC8, PA11, 13, 4);
//MagneticSensorI2C sensor2 = MagneticSensorI2C(0x60, 14, 0xFE, 8);


BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(9, 3, 11, 12);  // solder pin 7 on driver and wire it to 12 on driver not the stm32(NOT 7!)
MagneticSensorI2C sensor1 = MagneticSensorI2C(0x44, 14, 0xFE, 8);


BLDCMotor motor0 = BLDCMotor(11);
BLDCDriver3PWM driver0 = BLDCDriver3PWM(5, 10, 6, 8);
MagneticSensorI2C sensor0 = MagneticSensorI2C(0x40, 14, 0xFE, 8);

void setup() {

  driver0.voltage_power_supply = 12;
  driver0.init();
  motor0.linkDriver(&driver0);
  sensor0.init();
  motor0.linkSensor(&sensor0);
  motor0.voltage_sensor_align = 7;
  motor0.controller = MotionControlType::torque;
  motor0.phase_resistance = 11.1; // 11.1 Ohms

  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  sensor1.init();
  motor1.linkSensor(&sensor1);
  motor1.voltage_sensor_align = 7;
  motor1.controller = MotionControlType::torque;
  motor1.phase_resistance = 11.1; // 11.1 Ohms
//
//  driver2.voltage_power_supply = 12;
//  driver2.init();
//  motor2.linkDriver(&driver2);
//  sensor2.init();
//  motor2.linkSensor(&sensor2);
//  motor2.voltage_sensor_align = 7;
//  motor2.controller = MotionControlType::torque;
//  motor2.phase_resistance = 11.1; // 11.1 Ohms

  motor0.init();
  motor0.initFOC();
//  motor0.initFOC(5.55, CW);


  motor1.init();
  motor1.initFOC();
//  motor1.initFOC(3.48, CCW);


//  motor2.init();
//  motor2.initFOC();
//  motor2.initFOC(5.31, CW);


  
  Serial.begin(115200);
  Serial.println("Bottom Motor 0");
  Serial.println("Sensor zero offset is:");
  Serial.println(motor0.zero_electric_angle, 4);
  Serial.println("Sensor natural direction is: ");
  Serial.println(motor0.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");

  Serial.println("Mid Motor 1");
  Serial.println("Sensor zero offset is:");
  Serial.println(motor1.zero_electric_angle, 4);
  Serial.println("Sensor natural direction is: ");
  Serial.println(motor1.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");
//
//  Serial.println("Top Motor 2");
//  Serial.println("Sensor zero offset is:");
//  Serial.println(motor2.zero_electric_angle, 4);
//  Serial.println("Sensor natural direction is: ");
//  Serial.println(motor2.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");

  Serial.println("To use these values provide them to the: motor.initFOC(offset, direction)");
  _delay(1000);
  Serial.println("If motor is not moving the alignment procedure was not successfull!!");
  Serial.print(motor0.zero_electric_angle, 4);
  Serial.print('\t');
  Serial.println(motor1.zero_electric_angle, 4);
//  Serial.print('\t');
//  Serial.println(motor2.zero_electric_angle, 4);
  Serial.print(motor0.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");
  Serial.print('\t');
  Serial.println(motor1.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");
//  Serial.print('\t');
//  Serial.println(motor2.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");
}


void loop() {
    
  // main FOC algorithm function
  motor0.loopFOC();
  motor1.loopFOC();
//  motor2.loopFOC();

  // Motion control function
  motor0.move(0.2);
//  Serial.print(motor0.shaftAngle());
  motor1.move(0.2);
//  Serial.print('\t');
//  Serial.println(motor1.shaftAngle());
//  motor2.move(0.2);
}
