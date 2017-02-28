/*
 * libwifibot.cpp
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

#include "libwifibot.h"

typedef int16_t       int16;
typedef uint16_t      uint16;

//
// Internal functions only used by the library
//
static int _crc(BYTE *adresse, int taille_max)
{
  int crc = 0xFFFF;
  int polynome = 0xA001;
  int cptOctet = 0;
  int cptBit = 0;
  int parity = 0;
  
  for (cptOctet = 0; cptOctet < taille_max ; cptOctet++)
    {
      crc ^= *(adresse + cptOctet);
      
      for (cptBit = 0; cptBit <= 7 ; cptBit++)
	{
	  parity = crc;
	  crc >>= 1;
	  if (parity % 2 == true) crc ^= polynome;
	}
    }
  return(crc);
}

template<typename T>
static T _abs(T v)
{
  if (v < 0)
    return -v;

  return v;
}

template<typename T>
static T _between(T min, T val, T max)
{
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

template<typename T>
static void _hexdump(const T *data, int size)
{
  /* dumps size bytes of *data to stdout. Looks like:
   * [0000] 75 6E 6B 6E 6F 77 6E 20
   *                  30 FF 00 00 00 00 39 00 unknown 0.....9.
   * (in a single line of course)
   */

  const T *p = data;
  T c;
  int n;
  char bytestr[4] = {0};
  char addrstr[10] = {0};
  char hexstr[ 16*3 + 5] = {0};
  char charstr[16*1 + 5] = {0};
  for(n=1;n<=size;n++) {
    if (n%16 == 1) {
      /* store address for this line */
      snprintf(addrstr, sizeof(addrstr), "%.4x",
               ((unsigned int)p-(unsigned int)data) );
    }
            
    c = *p;
    if (isalnum(c) == 0) {
      c = '.';
    }

    /* store hex str (for left side) */
    snprintf(bytestr, sizeof(bytestr), "%02X ", *p);
    strncat(hexstr, bytestr, sizeof(hexstr)-strlen(hexstr)-1);

    /* store char str (for right side) */
    snprintf(bytestr, sizeof(bytestr), "%c", c);
    strncat(charstr, bytestr, sizeof(charstr)-strlen(charstr)-1);

    if(n%16 == 0) { 
      /* line completed */
      printf("[%4.4s]   %-50.50s  %s\n", addrstr, hexstr, charstr);
      hexstr[0] = 0;
      charstr[0] = 0;
    } else if(n%8 == 0) {
      /* half line: add whitespaces */
      strncat(hexstr, "  ", sizeof(hexstr)-strlen(hexstr)-1);
      strncat(charstr, " ", sizeof(charstr)-strlen(charstr)-1);
    }
    p++; /* next byte */
  }

  if (strlen(hexstr) > 0) {
    /* print rest of buffer if not empty */
    printf("[%4.4s]   %-50.50s  %s\n", addrstr, hexstr, charstr);
  }
}



//
// Library namespace
//
namespace wifibot
{

  // Serial class
  Serial::Serial()
    : _handle (0)
  {
    
  }
  
  Serial::~Serial()
  {
    close();
  }

  bool Serial::open(std::string device, int baudrate)
  {
    struct termios options;
    int bitrate;

    if (_handle)
      return false;

    // Only handle 9600 and 19200 baudrates.
    bitrate = (baudrate == 9600) ? B9600 : (baudrate == 19200) ? B19200 : 0;
    if (!bitrate)
      {
	std::cerr << "ERROR : Serial " << baudrate << " baudrate unsupported!" << std::endl;
	return false;
      }

    _handle = ::open(device.c_str(), O_RDWR | O_NOCTTY);

    if (_handle < 0)
      {
	_handle = 0;
	return false;
      }

    tcgetattr(_handle, &options);

    options.c_cflag = bitrate | CS8 | CREAD | CLOCAL;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;

    options.c_cc[VMIN]  = 1;
    options.c_cc[VTIME] = 0;

    tcsetattr(_handle, TCSANOW, &options);

    return true;
  }


  bool Serial::close()
  {
    if (!_handle)
      return false;

    ::close(_handle);
    _handle = 0;

    return true;
  }

  bool Serial::flush()
  {
    if (!_handle)
      return false;

    return tcflush(_handle, TCIFLUSH) == 0;
  }

  bool Serial::read(BYTE *pBuffer, int size, int *pHasRead)
  {
    if (!_handle)
      return false;
    
    //_hexdump(pBuffer, size);

    *pHasRead = ::read(_handle, pBuffer, size);

    if (*pHasRead < 0)
      return false;

    return true;
  }

  bool Serial::write(const BYTE *pBuffer, int size, int *pHasWritten)
  {
    if (!_handle)
      return false;

    //_hexdump(pBuffer, size);

    *pHasWritten = ::write(_handle, pBuffer, size);

    if (*pHasWritten < 0)
      return false;

    return true;
  }

  // timeout in ms
  bool Serial::timeout(int timeout)
  {
    fd_set rfds;
    struct timeval tv;
    int retval;
    
    FD_ZERO(&rfds);
    FD_SET(_handle, &rfds);
    
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout * 1000) - (tv.tv_sec * 1000 * 1000);
    retval = select(_handle + 1, &rfds, NULL, NULL, &tv);
    if (retval > 0)
      return true;
    
    if (retval == 0)
      return false;
    
    return false;
  }


  // Wifibot protocol class
  Protocol::Protocol()
    : _ticsPerMeter (5312.0)
    , _loopControl (true)
    , _loopControlSpeed (50)
  {
    for (int i = 0; i < W_RELAYS_NUMBER; i++)
      _pRelays[i] = false;

    fillHeader();
  }

  Protocol::~Protocol()
  {
  }

  void Protocol::fillHeader()
  {
    memset(_pBufferIn, 0, W_BUFFER_IN);
    memset(_pBufferOut, 0, W_BUFFER_OUT);

    _pBufferIn[0] = 0xff;
  }

  BYTE *Protocol::getBufferIn()
  {
    return _pBufferIn;
  }

  BYTE *Protocol::getBufferOut()
  {
    return _pBufferOut;
  }

  bool Protocol::process()
  {
    if (_pBufferOut[0] != 0xff)
      return false;

    BYTE *p = _pBufferOut + 1;

    _data.speedFrontLeft =   (p[1] << 8) + (p[0]);
    _data.voltage =          (int)p[2] / 10.0;
    _data.adc[0] =           p[3];
    _data.adc[1] =           p[4];
    _data.odometryLeft =     (p[8] << 24) + (p[7] << 16) + (p[6] << 8) + (p[5]);

    _data.speedFrontRight =  (p[10] << 8) + (p[9]);
    _data.adc[2] =            p[11];
    _data.adc[3] =            p[12];
    _data.odometryRight =    (p[16] << 24) + (p[15] << 16) + (p[14] << 8) + (p[13]);

    _data.current =          (int)p[17] * 0.194 - 37.5; // Need to know the formula ...
    _data.version =          p[18]; 
    
    if (_data.speedFrontLeft > 0x7fff)
      _data.speedFrontLeft = _data.speedFrontLeft - 0xffff;

    if (_data.speedFrontRight > 0x7fff)
      _data.speedFrontRight = _data.speedFrontRight - 0xffff;

    _data.odometryLeft = _data.odometryLeft / _ticsPerMeter;
    _data.odometryRight = _data.odometryRight / _ticsPerMeter;

    // _data.speedFrontLeft = (1000.0 * _data.speedFrontLeft) / (_loopControlSpeed * _ticsPerMeter);
    // _data.speedFrontRight = (1000.0 * _data.speedFrontRight) / (_loopControlSpeed * _ticsPerMeter);
    // Seems the firmware do the loop control speed conversion
    _data.speedFrontLeft = (1000.0 * _data.speedFrontLeft) / (50 *_ticsPerMeter);
    _data.speedFrontRight = (1000.0 * _data.speedFrontRight) / (50 * _ticsPerMeter);

    return true;
  }

  driverData Protocol::getData()
  {
    return _data;
  }

  bool Protocol::setTicsPerMeter(double tpm)
  {
    _ticsPerMeter = tpm;

    return true;
  }

  bool Protocol::enableLoopControl(bool enable)
  {
    _loopControl = enable;
    return _loopControl;
  }

  double Protocol::loopControlSpeed(double speed)
  {
    int s = speed * 1000.0;
    if (s != 10 && s != 50)
      return _loopControlSpeed / 1000.0;

    _loopControlSpeed = s;
    return _loopControlSpeed / 1000.0;
  }

  void Protocol::setRelays(const bool *pRelays)
  {
    for (int i = 0; i < W_RELAYS_NUMBER; i++)
      _pRelays[i] = pRelays[i];
  }

  bool Protocol::setSpeeds(double left, double right)
  {
    //    int16 sl = (left * _loopControlSpeed * _ticsPerMeter) / 1000.0;
    //    int16 sr = (right * _loopControlSpeed * _ticsPerMeter) / 1000.0;
    // Seems the firmware do the loop control speed conversion
    int16 sl = (left * 50 * _ticsPerMeter) / 1000.0;
    int16 sr = (right * 50 * _ticsPerMeter) / 1000.0;
    BYTE c = 0;

    sl = _between((int16)-W_SPEED_MAX, sl, (int16)W_SPEED_MAX);
    sr = _between((int16)-W_SPEED_MAX, sr, (int16)W_SPEED_MAX);

    if (_loopControl)
      {
	c += 0x80;                // [10000000b] = control loop for left wheel
	c += 0x20;                // [00100000b] = control loop for right wheel
      }

    if (sl > 0)
      c += 0x40;                  // [01000000b] = forward

    if (sr > 0)
      c += 0x10;                  // [00010000b] = forward

    if (_loopControlSpeed == 10)  // [00001000b] = pid at 10ms (!= 50ms)
      c += 0x08;

    if (_pRelays[2])               // [00000100b] = relay on
      c += 0x04;
    if (_pRelays[1])               // [00000010b] = relay on
      c += 0x02;
    if (_pRelays[0])               // [00000001b] = relay on
      c += 0x01;

    sl = _abs(sl);
    sr = _abs(sr);

    fillHeader();
    _pBufferIn[1] = 0x07; // size without checksum
    _pBufferIn[2] = sl >> 0;
    _pBufferIn[3] = sl >> 8;
    _pBufferIn[4] = sr >> 0;
    _pBufferIn[5] = sr >> 8;
    _pBufferIn[6] = c;

    uint16 crc = _crc(_pBufferIn + 1, _pBufferIn[1] - 1);
    _pBufferIn[7] = crc >> 0;
    _pBufferIn[8] = crc >> 8;

    return true;
  }

  bool Protocol::setPid(double p, double i, double d)
  {
    BYTE kp, ki, kd;

    kp = (BYTE)_between(0.0, p * 100.0, 255.0);
    ki = (BYTE)_between(0.0, i * 100.0, 255.0);
    kd = (BYTE)_between(0.0, d * 100.0, 255.0);

    fillHeader();
    _pBufferIn[1] = 0x09;
    _pBufferIn[2] = 0;
    _pBufferIn[3] = 0;
    _pBufferIn[4] = kp;
    _pBufferIn[5] = ki;
    _pBufferIn[6] = kd;
    _pBufferIn[7] = (BYTE)(W_SPEED_MAX >> 0);
    _pBufferIn[8] = (BYTE)(W_SPEED_MAX >> 8);

    uint16 crc = _crc(_pBufferIn + 1, _pBufferIn[1] - 1);
    _pBufferIn[9] = crc >> 0;
    _pBufferIn[10] = crc >> 8;

    return true;
  }



  Frame::Frame(int framesize)
  {
    _frameSize = framesize;
    _bufferSize = _frameSize * 2;

    _bufferIndex  = 0;

    _pBuffer = new BYTE[_bufferSize];
    memset(_pBuffer, 0, _bufferSize);
  }

  Frame::~Frame()
  {
    delete[] _pBuffer;
  }

  void Frame::append(const BYTE *pBuffer, int size)
  {
    for (int i = 0; i < size; i++)
      {
	_pBuffer[_bufferIndex++] = pBuffer[i];
	if (_bufferIndex == _bufferSize)
	  _bufferIndex = 0;
      }
  }

  bool Frame::getFrame(BYTE *pFrame)
  {
    int b = _bufferIndex - 1;

    for (int i = 0; i < _bufferSize; i++)
      {
	if (b < 0) b = _bufferSize - 1;

	if (_pBuffer[b] == 0xff)
	  {
	    for (int j = 0; j < _frameSize; j++)
	      {
		pFrame[j] = _pBuffer[(b + j) % _bufferSize];
	      }

	    uint16 rcrc = (pFrame[_frameSize - 1] << 8) + pFrame[_frameSize - 2];
	    uint16 ccrc = _crc(pFrame + 1, _frameSize - 3);

	    if (rcrc == ccrc)
	      return true;
	  }
	b--;
      } 

    return false;
  }


  Driver::Driver(std::string device)
  {
    _pFrame = new Frame(22);
    _serial.open(device);
  }

  Driver::~Driver()
  {
    _serial.close();
    delete _pFrame;
  }

  // This function read data from the serial device
  // until a complete frame is received.
  // A frame is defined by a length (see Frame constructor)
  // and a checksum as defined by the wifibot protocol.
  bool Driver::processRead()
  {
    int r;

    while (42)
      {
	// If no data is received within 1sec, exit.
	if (!_serial.timeout(1000))
	  {	
	    std::cerr << "Error during serial.timeout" << std::endl;
	    return false;
	  }

	BYTE buffer[32];
	// If an error occures on the serial line, exit.
	if (!_serial.read(buffer, 32, &r))
	  {
	    std::cerr << "Error during serial.read" << std::endl;
	    return false;
	  }
      
	// Fill the frame with the freshly received data
	_pFrame->append(buffer, r);
	
	// Check if we got a complete frame
	if (_pFrame->getFrame(_protocol.getBufferOut()))
	  break ;
      }

    // Fill the wifibot data structure with this frame
    return _protocol.process();
  }

  // Send data through the serial device
  bool Driver::processWrite()
  {
    int w;
    const BYTE *pB = _protocol.getBufferIn();

    if (!_serial.write(pB, pB[1] + 2, &w))
      return false;

    if (w != pB[1] + 2)
      return false;

    return true;
  }

  // Read and print the wifibot latest wifibot data 
  driverData Driver::readData()
  {
    // Read the serial device, get a frame and fill the data structure
    if (!processRead())
      {
	std::cerr << "Error in readData()" << std::endl;
      }
    
    // Get the structure
    return _protocol.getData();
  }

  // Set and apply left and right speeds
  bool Driver::setSpeeds(double speedLeft, double speedRight)
  {
    _protocol.setSpeeds(speedLeft, speedRight);

    if (!processWrite())
      {
	std::cerr << "Error in setSpeeds()" << std::endl;
	return false;
      }

    return true;
  }

  // Set and apply PID factors
  bool Driver::setPid(double kp, double ki, double kd)
  {
    _protocol.setPid(kp, ki, kd);

    if (!processWrite())
      {
	std::cerr << "Error in setPid()" << std::endl;
        return false;
      }
    return true;
  }

  // Set relays values (will be applied on the next setSpeed() call)
  bool Driver::setRelays(bool r1, bool r2, bool r3)
  {
    bool pRelays[W_RELAYS_NUMBER];

    pRelays[0] = r1;
    pRelays[1] = r2;
    pRelays[2] = r3;
    _protocol.setRelays(pRelays);

    return true;
  }

  // Set loop control speed (will be applied on the next setSpeed() call)
  bool Driver::loopControlSpeed(double speed)
  {
    double s = _protocol.loopControlSpeed(speed);
    
    return (s == speed);
  }

  // Enable/disable loop control (will be applied on the next setSpeed() call)
  bool Driver::enableLoopControl(bool enable)
  {
    return _protocol.enableLoopControl(enable);
  }

  // Set tics/meter value. Used for units conversion
  bool Driver::setTicsPerMeter(double tpm)
  {
    return _protocol.setTicsPerMeter(tpm);
  }

  // Return the library version
  double Driver::getVersion()
  {
    return W_VERSION;
  }

}
