/*
 * libwifibot.h
 *
 * Copyright (c) 2012, Jean Charles Mammana. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301  USA
 */

#include <stdlib.h>
#include <stdio.h>
#include <termios.h> 
#include <unistd.h>
#include <iostream>
#include <sys/stat.h> 
#include <fcntl.h>
#include <string.h>
#include <stdint.h>

typedef unsigned char BYTE;

#define W_BUFFER_IN     64
#define W_BUFFER_OUT    64
#define W_RELAYS_NUMBER 3
#define W_SPEED_MAX     360
#define W_VERSION       1.0

// Wifibot library namespace
namespace wifibot
{
  typedef struct sDriverData
  {
    double speedFrontLeft;    // Speed Left in m/s (using the tics/meter value)
    double speedFrontRight;   // Speed Right in m/s (using the tics/meter value)
    double odometryLeft;      // Odometry Left in meters (using the tics/meter value)
    double odometryRight;     // Odometry Right in meters (using the tics/meter value)
    double current;           // Robot current in I
    double voltage;           // Robot voltage in V
    int adc[4];               // Raw adc values
    int version;              // Firmware version
  } driverData;
  

  // Class that handle serial port
  class Serial
  {
  private:
    int _handle;
    
  public:
    Serial();
    ~Serial();
    
    bool open(std::string device, int baudrate = 19200);
    bool close();
    bool flush();
    bool read(BYTE *pBuffer, int size, int *pHasRead);
    bool write(const BYTE *pBuffer, int size, int *pHasWritten);
    bool timeout(int timeout); // in ms
  };
  
  // Class that handle wifibot protocol
  class Protocol
  {
  private:
    BYTE _pBufferIn[W_BUFFER_IN];
    BYTE _pBufferOut[W_BUFFER_OUT];
    driverData _data;
    double _ticsPerMeter;
    bool _loopControl;
    int _loopControlSpeed;
    bool _pRelays[W_RELAYS_NUMBER];
    
    void fillHeader();
    
  public:
    Protocol();
    ~Protocol();
    
    BYTE *getBufferIn();
    BYTE *getBufferOut();
    
    // Parse buffer and fill data structure
    bool process();
    
    // Return data from firmware
    driverData getData();
    
    // Set the tics per meter value
    bool setTicsPerMeter(double tpm);
    
    // Enable/Disable control loop (default:true)
    // Return the current control loop status
    // Applied on next setSpeeds()
    bool enableLoopControl(bool enable);
    
    // Set control loop speed (default:0.05 second)
    // Only 0.05 and 0.01 are valid values
    // Return the current control loop speed
    // Applied on next setSpeeds()
    double loopControlSpeed(double speed);
    
    // Set relays status. (default: all off)
    // Applied on next setSpeeds()
    void setRelays(const bool *pRelays);
    
    // Set left and right speed in m/s
    bool setSpeeds(double left, double right);
    
    // Set PID factors between 0 and 2.55
    bool setPid(double p, double i, double d);
  };
  

  // Class that parses buffer and get a fixed sized frame
  // Used for input serial buffer.
  class Frame
  {
  private:
    BYTE *_pBuffer;
    int _bufferSize;
    int _bufferIndex;
    int _frameSize;
    
  public:
    Frame(int framesize);
    ~Frame();
    
    void append(const BYTE *pBuffer, int size);
    bool getFrame(BYTE *pFrame);
  };
  
  // High level class.
  // Mainly used to explain howto deal with other classes
  class Driver
  {
  private:
    Serial _serial;
    Frame *_pFrame;
    Protocol _protocol;
    
    bool processRead();  // Execute the serial read action
    bool processWrite(); // Execute the serial write action
    
  public:
    Driver(std::string device);                            // device is the serial device (eg:/dev/ttyS0)
    ~Driver();
    
    driverData readData();
    bool setSpeeds(double speedLeft, double speedRight);   // Set speeds. Call this function at least every 0.25s
    bool setPid(double kp, double ki, double kd);          // Set PID factor values for left and right wheels
    bool setRelays(bool r1, bool r2, bool r3);             // Turn on/off wifibots relays
    bool loopControlSpeed(double speed);                   // Change the loop control speed
    bool enableLoopControl(bool enable);                   // Enable/disable the loop control
    bool setTicsPerMeter(double tpm);                      // Set the tics/meter value
    double getVersion();                                   // Return the library version
  };
}
