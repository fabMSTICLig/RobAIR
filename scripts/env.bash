export ROBAIR_IP=`ip route get 8.8.8.8 | awk 'NR==1 {print $NF}'`
export PATH="$PATH:$ROBAIR_HOME/scripts/"
source /opt/ros/kinetic/setup.bash
source $ROBAIR_HOME/catkin_ws/devel/setup.bash
