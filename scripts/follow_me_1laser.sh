#!/bin/bash


if [ -z $ROBAIR_HOME ]; then
  export ROBAIR_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  source "$ROBAIR_HOME/scripts/env.bash"
fi

rosrun follow_me_1laser one_person_detector_and_tracker_follow_me_1laser_node_stable &
rosrun follow_me_1laser robot_moving_follow_me_1laser_node &
rosrun follow_me_1laser obstacle_detection_follow_me_1laser_node &
rosrun follow_me_1laser action_follow_me_1laser_node_stable &
