#include <RPLidar.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define RPLIDAR_MOTOR 5
#define NEO_PIN       3
#define NUMPIXELS     24
#define I2C_ADDR      0x4B
RPLidar lidar;
Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);
const int cycles = 1;
                    
float minDistance = 10000;
float angleAtMinDist = 0;
byte send_distance = 0;
byte send_angle = 0;

void setup() {
  Serial.begin(115200);  //not required
  lidar.begin(Serial);

  pinMode(RPLIDAR_MOTOR, OUTPUT);
  pinMode(NEO_PIN, OUTPUT);
  pixels.begin();

  Wire.begin(I2C_ADDR);  // join i2c bus with address #4
  Wire.onRequest(requestEvent); // request event
}

int cur_cycle = 0;
void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;

    if (lidar.getCurrentPoint().startBit) {
      cur_cycle += 1;
      if (cur_cycle == cycles){
        cur_cycle = 0;
        displayColor(angleAtMinDist, minDistance);
        minDistance = 10000;
        angleAtMinDist = 0;
      } else {
        if ( distance > 0 &&  distance < minDistance) {
          minDistance = distance;
          angleAtMinDist = angle;
       }
      } 
    } else {
       if ( distance > 0 &&  distance < minDistance) {
          minDistance = distance;
          angleAtMinDist = angle;
       }
    }
    
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       //detected...
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}



void displayColor(float angle, float distance)
{
    byte neoID = round(angle*23.0f/360.0f);
    if (neoID == 24) neoID = 23;
    send_angle = byte(angle*255.0f/360.0f);
//    send_distance = (distance>500.0)?0:(255-distance*255/500);
    byte dis = constrain(distance*0.2, 0, 255);
    const float minVal = 0;
    const float maxVal = 255;
    const float reading = max(min(send_distance, maxVal), minVal);
    const float ratio = 2.0f * (reading - minVal) / ((maxVal - minVal)+0.000001);
    const byte r_ = (max(0.0f, 255.0f*(1.0f - ratio)));
    const byte b_ = (max(0.0f, 255.0f*(ratio - 1.0f)));
    const byte g_ = 255 - b_ - r_;   
    pixels.clear();   
    pixels.setPixelColor(neoID, r_, g_, b_);
    pixels.show();    
}

void requestEvent()
{
  Wire.write(send_angle);
  Wire.write(send_distance);
}
