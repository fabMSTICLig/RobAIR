#!/bin/bash
roslaunchpid=`pgrep -f roslaunch`
kill -SIGINT $roslaunchpid
