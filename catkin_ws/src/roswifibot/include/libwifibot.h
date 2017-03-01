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

#include "ros/ros.h"
#include <robairmain/MotorsInfo.h>

// Wifibot library namespace
namespace wifibot
{
  typedef struct sDriverData
  {
    double speedFrontLeft;    // Speed Left in m/s (using the tics/meter value)
    double speedFrontRight;   // Speed Right in m/s (using the tics/meter value)
    double odometryLeft;      // Odometry Left in meters (using the tics/meter value)
    double odometryRight;     // Odometry Right in meters (using the tics/meter value)
    sDriverData() :
      speedFrontLeft(0.0),
      speedFrontRight(0.0),
      odometryLeft(0.0),
      odometryRight(0.0) {}
  } driverData;

  // High level class.
  // Mainly used to explain howto deal with other classes
  class Driver
  {
  private:
    double _ticsPerMeter;
    driverData _data;
    ros::Subscriber _subMotors;

    void motorsCallback(const robairmain::MotorsInfo& msg);

  public:
    Driver(ros::NodeHandle nh);

    driverData readData();
    void setTicsPerMeter(double tpm);                // Set the tics/meter value
  };
}
