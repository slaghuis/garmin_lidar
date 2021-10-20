// Copyright (c) 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* **********************************************************************
 * Library to drive the Garmin Lidar Lite v3
 * Reduced functionality, just enough to take a range measurement
 * Shamelessly translated from https://github.com/garmin/LIDARLite_RaspberryPi_Library
 * because their code used strange types (WTF is __u8, and display some errors in the
 * i2crite function line 301.  I think this was microcontroller code that was copied
 * and pasted.
 * ***********************************************************************/

#include <garmin_lidar/garmin_lidar.h>

// Constructor ////////////////////////////////////////////
GarminLidar::GarminLidar()
  : address(AddressDefault)
{ }

// Public Methods ////////////////////////////////////////
//Open the i2c port for reading and writing
//and initialise the sensor
bool GarminLidar::init() {
  // Open it i2c port
  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
      return false;
  }

  // Setup flow control
  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    return false;
  }
  
  return true;
}  

/*------------------------------------------------------------------------------
  Configure
  Selects one of several preset configurations.
  Parameters
  ------------------------------------------------------------------------------
  mode:  Default 0.
    0: Default mode, balanced performance.
    1: Short range, high speed. Uses 0x1d maximum acquisition count.
    2: Default range, higher speed short range. Turns on quick termination
        detection for faster measurements at short range (with decreased
        accuracy)
    3: Maximum range. Uses 0xff maximum acquisition count.
    4: High sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for high sensitivity and noise.
    5: Low sensitivity detection. Overrides default valid measurement detection
        algorithm, and uses a threshold value for low sensitivity and noise.

------------------------------------------------------------------------------*/
void GarminLidar::configure(SensorMode mode)
{
    uint8_t sigCountMax;
    uint8_t acqConfigReg;
    uint8_t refCountMax;
    uint8_t thresholdBypass;

    switch (mode)
    {
        case Default: // Default mode, balanced performance
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case ShortRange: // Short range, high speed
            sigCountMax     = 0x1d;
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case DefaultRange: // Default range, higher speed short range
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x00;
            refCountMax     = 0x03;
            thresholdBypass = 0x00; // Default
            break;

        case MaxRange: // Maximum range
            sigCountMax     = 0xff;
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x00; // Default
            break;

        case HighSens: // High sensitivity detection, high erroneous measurements
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0x80;
            break;

        case LowSens: // Low sensitivity detection, low erroneous measurements
            sigCountMax     = 0x80; // Default
            acqConfigReg    = 0x08; // Default
            refCountMax     = 0x05; // Default
            thresholdBypass = 0xb0;
            break;

        case ShortRangeErr: // Short range, high speed, higher error
            sigCountMax     = 0x04;
            acqConfigReg    = 0x01; // turn off short_sig, mode pin = status output mode
            refCountMax     = 0x03;
            thresholdBypass = 0x00;
            break;
    }

    writeReg(LLv3_SIG_CNT_VAL,   sigCountMax);
    writeReg(LLv3_ACQ_CONFIG,    acqConfigReg);
    writeReg(LLv3_REF_CNT_VAL,   refCountMax);
    writeReg(LLv3_THRESH_BYPASS, thresholdBypass);
}

/*------------------------------------------------------------------------------
  Take Range
  Initiate a distance measurement by writing to register 0x00.
  ----------------------------------------------------------------------------*/
void GarminLidar::takeRange()
{
    uint8_t commandByte = 0x04;

    writeReg(LLv3_ACQ_CMD, commandByte);
} 

/*------------------------------------------------------------------------------
  Wait for Busy Flag
  Blocking function to wait until the Lidar Lite's internal busy flag goes low
------------------------------------------------------------------------------*/
void GarminLidar::waitForBusy()
{
    uint8_t  busyFlag;

    do  // Loop until device is not busy
    {
        busyFlag = getBusyFlag();
    } while (busyFlag);
}
             

/*------------------------------------------------------------------------------
  Get Busy Flag
  Read BUSY flag from device registers. Function will return 0x00 if not busy.
------------------------------------------------------------------------------*/
uint8_t GarminLidar::getBusyFlag()
{
    uint8_t  status_byte = 0;
    uint8_t  busy_flag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    status_byte = readReg(LLv3_STATUS);

    // STATUS bit 0 is busyFlag
    busy_flag = status_byte & 0x01;

    return busy_flag;
}
 
/*------------------------------------------------------------------------------
  Read Distance
  Read and return result of distance measurement.
------------------------------------------------------------------------------*/
uint16_t GarminLidar::readDistance()
{
    uint16_t   distance;
  
    distance = readReg16Bit((LLv3_DISTANCE | 0x80));

    return distance;
}
             
             
// Private Methods ////////////////////////////////////////

//Write an 8-bit register
void GarminLidar::writeReg(uint8_t reg, uint8_t value)
{
  uint8_t writeBuffer[2];

  writeBuffer[0] = reg;
  writeBuffer[1] = value;

  auto result = write(fd, writeBuffer, sizeof(writeBuffer));
  last_status = (result > 0) ? 0 : 1;
}

// Read an 8-bit register
uint8_t GarminLidar::readReg(uint8_t reg)
{
  uint8_t  writeBuffer[1];

  writeBuffer[0] = reg;

  if( write(fd, writeBuffer, sizeof(writeBuffer)) < (long int)sizeof(writeBuffer)) {
    //something went wrong
    std::cout << "DEBUG: ERROR on write to i2c" << std::endl;
    return 0x00;
  }

  uint8_t readBuffer[1];
  read(fd, readBuffer, sizeof(readBuffer));

  return readBuffer[0];
}

//Read 16 bits from a register
uint16_t GarminLidar::readReg16Bit(uint8_t reg) {

  uint8_t  writeBuffer[1];
  writeBuffer[0] = reg;

  if( write(fd, writeBuffer, sizeof(writeBuffer)) < (long int)sizeof(writeBuffer)) {
    //something went wrong
    std::cout << "ERROR on write to i2c" << std::endl;
    return 0x0000;
  }

  uint8_t readBuffer[2];
  if( read(fd, readBuffer, sizeof(readBuffer)) < (long int)sizeof(readBuffer)) {
    std::cout << "Error on read from i2c" << std::endl;
    return 0x0000;
  }

  uint16_t value = (uint16_t)readBuffer[0] << 8;  //value high byte
  value |= readBuffer[1];                         // value low byte

  return value;
}
