#!/bin/bash

kill -9 `pgrep -f robairmain/proxy`
stoprobair.bash
sleep 3

#export ROBAIR_HOME=/home/robair/Rob-AIR
#source $ROBAIR_HOME/scripts/env.bash

roslaunch robairmain robair.launch
