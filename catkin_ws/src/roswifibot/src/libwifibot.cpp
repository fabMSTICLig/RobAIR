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

#include "libwifibot.h"

//
// Library namespace
//
namespace wifibot
{
  Driver::Driver(ros::NodeHandle nh)
    : _data(), _ticsPerMeter(1)
  {
    _subMotors = nh.subscribe("/motors_info", 1, &Driver::motorsCallback, this);
  }

  void Driver::motorsCallback(const robairmain::MotorsInfo& msg)
  {
    _data.speedFrontLeft = (double)msg.speedL / (60.0 * _ticsPerMeter);
    _data.speedFrontRight = (double)msg.speedR / (60.0 * _ticsPerMeter);
    _data.odometryLeft = (double)msg.countL / _ticsPerMeter;
    _data.odometryRight = (double)msg.countR / _ticsPerMeter;
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
