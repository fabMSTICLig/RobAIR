#!/bin/bash

for pid in `ps ax | grep "catkin_ws/devel/lib/follow_me_1laser/" | grep -v grep |  awk '{print $1}'`
	do kill -SIGINT "$pid"
done
