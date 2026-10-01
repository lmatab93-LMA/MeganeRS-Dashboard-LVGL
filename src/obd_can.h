#ifndef OBD_CAN_H
#define OBD_CAN_H

#include <Arduino.h>

struct VehicleData {
  bool valid = false;
  int rpm = 0;
  int speedKph = 0;
  int coolantTempC = 0;
  int mapKpa = 100;
  int intakeTempC = 0;
};

bool obdCanInit();
void obdCanRequestPid(uint8_t pid);
void obdCanPoll(VehicleData &data);
void obdCanDemoValues(VehicleData &data);

#endif