#ifndef GARMIN_LIDAR_H
#define GARMIN_LIDAR_H

#include <stdio.h>
#include <iostream>  // for output to std::cout
#include <linux/i2c-dev.h> // for the ioctl() function
#include <unistd.h> // for the read() and write() function
#include <fcntl.h> // for the open() function include <stdio.h>
#include <string.h> // for the strlen() function
#include <stdlib.h> // for exit
#include <sys/ioctl.h>
#include <errno.h>
#include <chrono>
#include <thread>

// LIDAR-Lite default I2C device address
#define AddressDefault 0x62

// LIDAR-Lite internal register addresses
#define LLv3_ACQ_CMD       0x00
#define LLv3_STATUS        0x01
#define LLv3_SIG_CNT_VAL   0x02
#define LLv3_ACQ_CONFIG    0x04
#define LLv3_DISTANCE      0x0f
#define LLv3_REF_CNT_VAL   0x12
#define LLv3_UNIT_ID_HIGH  0x16
#define LLv3_UNIT_ID_LOW   0x17
#define LLv3_I2C_ID_HIGH   0x18
#define LLv3_I2C_ID_LOW    0x19
#define LLv3_I2C_SEC_ADR   0x1a
#define LLv3_THRESH_BYPASS 0x1c
#define LLv3_I2C_CONFIG    0x1e
#define LLv3_COMMAND       0x40
#define LLv3_CORR_DATA     0x52
#define LLv3_ACQ_SETTINGS  0x5d

class GarminLidar {
  public:
    GarminLidar();
  
    enum SensorMode { 
      Default    = 0,    // Default mode, balanced performance
      ShortRange = 1,    // Short range, high speed
      DefaultRange = 2,  // Default range, higher speed short range
      MaxRange = 3,      // Maximum range
      HighSens = 4,      // High sensitivity detection, high erroneous measurements
      LowSens = 5,       // Low sensitivity detection, low erroneous measurements   
      ShortRangeErr = 6  // Short range, high speed, higher error
    };
  
    bool init();
    void configure(SensorMode mode);
    void takeRange();
    void waitForBusy();
    uint8_t getBusyFlag();
    uint16_t readDistance();
  
  private:
    
    int fd;  //File Descriptor
    uint8_t address;  //I2C address 
    uint8_t last_status;
  
    // Generic i2c Functions
    void writeReg(uint8_t reg, uint8_t value);

    uint8_t readReg(uint8_t reg);
    uint16_t readReg16Bit(uint8_t reg);    
};

#endif  // GARMIN_LIDAR_H
