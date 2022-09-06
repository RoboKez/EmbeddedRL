#ifndef MEMORYSD_H
#define MEMORYSD_H

// Reference: This file is taken from the default arduino library SD_test example, the read file was changed for json
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "json.hpp"
using json = nlohmann::json;

nlohmann::json readJsonFile(fs::FS &fs, const char * path);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void deleteFile(fs::FS &fs, const char * path);

#endif // MEMORYSD_H
