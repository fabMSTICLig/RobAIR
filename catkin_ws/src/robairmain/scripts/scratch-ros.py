#!/usr/bin/env python

import rospy
import scratch
from threading import Thread
from robairmain.msg import MotorsCmd
from std_msgs.msg import Int8
from std_msgs.msg import Bool

client = scratch.Scratch(host='0.0.0.0',port=42001)
mcmd = MotorsCmd()

angleHead = 180
last_angleHead = 100

last_speedL = 0
last_speedR = 0

def callback_bumper_rear(data):
  rospy.loginfo(data.data)
  client.sensorupdate({'bumper_rear':data.data})

def callback_bumper_front(data):
  rospy.loginfo(data.data)
  client.sensorupdate({'bumper_front':data.data})

def callback_touch_left(data):
  rospy.loginfo(data.data)
  client.sensorupdate({'touch_left':data.data})

def callback_touch_right(data):
  rospy.loginfo(data.data)
  client.sensorupdate({'touch_right':data.data})


def listener():
  rospy.Subscriber("bumper_rear",Bool,callback_bumper_rear)
  rospy.Subscriber("bumper_front",Bool,callback_bumper_front)
  rospy.Subscriber("touch_left",Bool,callback_touch_left)
  rospy.Subscriber("touch_right",Bool,callback_touch_right)
  
  rospy.spin()

def listen():
  while True:
    try:
      yield client.receive()
    except scratch.ScratchError:
      raise StopIteration

def update_scratch_msg():
  global angleHead
  for msg in listen():
    if msg[0] == 'broadcast':
      print msg[1]
    elif msg[0] == 'sensor-update':
      if msg[1].has_key('speedL'):
         mcmd.speedL = msg[1]['speedL']
         print "m1:",mcmd.speedL
      if msg[1].has_key('speedR'):
         mcmd.speedR = msg[1]['speedR']
         print " m2:",mcmd.speedR
      if msg[1].has_key('cmdHead'):
         angleHead = msg[1]['cmdHead']
         print " angleHead:",angleHead

def publisher():

  global last_speedL
  global last_speedR
  global last_angleHead
  pubMotors = rospy.Publisher('cmdmotors',MotorsCmd, queue_size=10)
  pubHead = rospy.Publisher('cmdhead',Int8, queue_size=10)
  rate = rospy.Rate(100)
  while not rospy.is_shutdown():
    if (mcmd.speedL != last_speedL) or (mcmd.speedR != last_speedR):
	rospy.loginfo(mcmd)
    	pubMotors.publish(mcmd)
	last_speedL = mcmd.speedL
        last_speedR = mcmd.speedR
    if  angleHead != last_angleHead:
        rospy.loginfo(angleHead)
        pubHead.publish(angleHead)
        last_angleHead = angleHead
        
    rate.sleep()
  thread_update_scratch.join()    


if __name__ == '__main__':
  try:    
    thread_update_scratch = Thread(None,update_scratch_msg,(),{})
    thread_update_scratch.start()
    print 'thread scratch message'
    rospy.init_node('scratchClient', anonymous=True)
    rospy.loginfo('Node initialized')
    print 'Node initialized'
    listen() #scartch message buffer
    listener() #suscriber des evenements sur le robot
    publisher() #publisher vers les moteurs
  except rospy.ROSInterruptException:
    print 'error'
    rospy.logerr('error')
    pass

