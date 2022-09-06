// SD ==============================================================================================
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <SimpleFOC.h> // Ref: SimpleFOC for encoders
#define LED_BUILTIN 13
#define I2C_ADDR_BRUSHLESS  2
#define I2C_ADDR_LIDAR  0x4B
#define ENCL_A 11
#define ENCL_B 10
#define ENCR_A 38
#define ENCR_B 33
//#define I2C_ADDR_LEFT_MAG  0x60
//#define I2C_ADDR_RIGHT_MAG 0x4C


void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

// DOTSTAR ==============================================================================================
#include <Adafruit_DotStar.h>
#define NUMPIXELS 128 
#define DATAPIN    17
#define CLOCKPIN   18
Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// BNO055 ==============================================================================================
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// WHEEELS ==============================================================================================
// create the encoder instances
Encoder encoderL = Encoder(ENCL_A, ENCL_B, 500);
void doL1() {  encoderL.handleA(); }
void doL2() {  encoderL.handleB(); }

Encoder encoderR = Encoder(ENCR_A, ENCR_B, 500);
void doR1() {  encoderR.handleA(); }
void doR2() {  encoderR.handleB(); }


void setup() {
  delay(1000);
  strip.begin(); // Initialize pins for output
  strip.setBrightness(10); // 10 min 100 max
  strip.show();  // Turn all LEDs off ASAP

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  delay(100);
  if(!SD.begin()){
      Serial.println("Card Mount Failed");
      return;
  } else {
    digitalWrite(LED_BUILTIN, HIGH); 
  }

  delay(1000);

  encoderL.init();
  encoderL.enableInterrupts(doL1, doL2);
  encoderR.init();
  encoderR.enableInterrupts(doR1, doR2);
  
  

//  sensorL.init();
//  sensorR.init();

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
}

void loop() {
  sendTorque(0.2, 0.2);
  for(int i=0; i<128; i++){
    singlePixel(i, 30, 100); 
    
    printPitch();
    printLidar();
    printWheels();
    Serial.println();
  }
  sendTorque(-0.2, -0.2);
  for(int i=0; i<128; i++){
    singlePixel(i,  0, 100);
    
    printPitch(); 
    printLidar();
    printWheels();
    Serial.println();
  }
}



// DOTSTAR ========================================================================================================
void singlePixel(int pix, int b, int wait) 
{
      strip.setPixelColor(pix, 40, 0, b);
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
}

// BNO055 =========================================================================================================
void printPitch()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("\tAPitch:");
  Serial.print(euler.z());
}

// LIDAR =========================================================================================================
void printLidar()
{
  const byte bytes_requested = 2;
  byte pos_[2] = {127, 127};
  Wire.requestFrom(byte(I2C_ADDR_LIDAR), bytes_requested);  // Request 2 bytes, change if state size changes!!!
  byte bytes_recieved;
  bytes_recieved = Wire.readBytes(pos_, bytes_requested);
  Serial.print("\tLidar_Ang:");
  Serial.print(float(pos_[0]));
  Serial.print("\tLidar_Pos:");
  Serial.print(float(pos_[1]));
}

// WHEELS ========================================================================================================
void printWheels()
{
  encoderL.update();
  encoderR.update();
  Serial.print("\tEncL:");
  Serial.print(encoderL.getVelocity());
  Serial.print("\tEncR:");
  Serial.print(encoderR.getVelocity());
}

// BRUSHLESS =====================================================================================================
void sendTorque(float left, float right)
{
  left = max(min(left, 1.0f), -1.0f);
  right = max(min(right, 1.0f), -1.0f);

  byte left_byte  = byte( left * 127.5f + 127.5f);
  byte right_byte  = byte( right * 127.5f + 127.5f);

  Wire.beginTransmission(I2C_ADDR_BRUSHLESS);     // transmit to device
  Wire.write(left_byte);                      // sends one byte
  Wire.write(right_byte);                     // sends one byte
  Wire.endTransmission();                // stop transmitting 
}
