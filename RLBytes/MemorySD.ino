// Reference: This file is taken directly from the standard SD_test example
#include "FS.h"
#include "SD.h"
#include "SPI.h"


void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
      while(1){
        
        Serial.println("mkdir failed");
        digitalWrite(LED_BUILTIN, LOW); 
        Serial.print(path);
        delay(1000);
      }   
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

nlohmann::json readJsonFile(fs::FS &fs, const char * path){
    Serial.printf("Reading json file: %s\n", path);
    File file = fs.open(path);
    size_t len = file.size();
    
    while(!file){
        digitalWrite(LED_BUILTIN, LOW);

        Serial.println("RL error: Failed to open file for reading, is the sd card in?");
        file.close();
        delay(200);
        file = fs.open(path);
        len = file.size();
    }
    digitalWrite(LED_BUILTIN, HIGH);
    String jsonStr = file.readString();
    file.close();
    json j = json::parse(jsonStr);
    return j;
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    while(!file){
        Serial.println("Failed to open file for writing");
        digitalWrite(LED_BUILTIN, LOW); 
        file.close();
        delay(200);
        file = fs.open(path, FILE_WRITE);
        delay(1000);
    }
    digitalWrite(LED_BUILTIN, HIGH);
    if(file.print(message)){
        Serial.println("File written");
        
    } else {
        while(1){
        Serial.println("Write failed");
        digitalWrite(LED_BUILTIN, LOW); 
        delay(1000);
        }
    }
    file.close();
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}
