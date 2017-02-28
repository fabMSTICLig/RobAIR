#!/usr/bin/env python                                                                                        
# Software License Agreement (BSD License)                                                                   
#                                                                                                            
# Copyright (c) 2009, Willow Garage, Inc.                                                                    
# All rights reserved.                                                                                       
#                                                                                                            
# Redistribution and use in source and binary forms, with or without                                         
# modification, are permitted provided that the following conditions                                         
# are met:                                                                                                   
#                      
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided                                         
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived                                         
#    from this software without specific prior written permission.
#                                                                                                            
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS                                          
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT                                         
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE                                            
# POSSIBILITY OF SUCH DAMAGE.
#
# Author Melonee Wise 

#import roslib; roslib.load_manifest('wifibot')
import rospy
import math
import sensor_msgs
import geometry_msgs

#from joy.msg import Joy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def mycallback(joy):
  vel = geometry_msgs.msg.Twist()

  vel.linear.x = (joy.axes[1] * 1.0);
  vel.angular.z = (joy.axes[0] * (3.14 * 2));

  pub.publish(vel)
  #rospy.loginfo(vel)

if __name__ == '__main__':

  name ='teleop'
  rospy.init_node(name)

 # linear = rospy.get_param('axis_linear', 1)
#  angular = rospy.get_param('axis_angular', 2)
#  l_scale = rospy.get_param('scale_linear', 50.0)
#  a_scale = rospy.get_param('scale_angular', 2.0)
 
  sub = rospy.Subscriber("joy", sensor_msgs.msg.Joy, mycallback)
  pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist)

  scale = 100
  rospy.spin()
