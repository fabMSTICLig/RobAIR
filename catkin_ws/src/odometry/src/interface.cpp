/*
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

#include "interface.h"

//
// Library namespace
//
namespace odometry
{
  Driver::Driver(ros::NodeHandle nh)
    : _data(), _ticsPerMeter(1), _lastCountL(0), _lastCountR(0),
    _overflowCountL(0), _overflowCountR(0)
  {
    _subMotors = nh.subscribe("/motors_info", 1, &Driver::motorsCallback, this);
  }

  void Driver::motorsCallback(const robairmain::MotorsInfo& msg)
  {
    detectOverflow(msg);

    _data.speedFrontLeft = (double)msg.speedL / (60.0 * _ticsPerMeter);
    _data.speedFrontRight = (double)msg.speedR / (60.0 * _ticsPerMeter);

    _data.odometryLeft =
      (double)accountForOverflows(msg.countL, false) / _ticsPerMeter;
    _data.odometryRight =
      (double)accountForOverflows(msg.countR, true) / _ticsPerMeter;
  }

  void Driver::detectOverflow(const robairmain::MotorsInfo& msg)
  {
    int dleft = msg.countL - _lastCountL,
        dright = msg.countR - _lastCountR;

    if (dleft < -64000)
      _overflowCountL += 1;
    else if (dleft > 64000)
      _overflowCountL -= 1;
    if (dright < -64000)
      _overflowCountR += 1;
    else if (dright > 64000)
      _overflowCountR -= 1;

    _lastCountL = msg.countL;
    _lastCountR = msg.countR;
  }

  int Driver::accountForOverflows(int32_t count, bool right)
  {
    int overflows = (right ? _overflowCountR : _overflowCountL);
    return count + overflows * 65536;
  }

  driverData Driver::readData()
  {
    return _data;
  }

  // Set tics/meter value. Used for units conversion
  void Driver::setTicsPerMeter(double tpm)
  {
    _ticsPerMeter = tpm;
  }
}
