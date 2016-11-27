export ROBAIR_IP=`ifconfig | grep 192.168 | awk -F'[: ]+' '{print $4}'`
if [[ -z $ROBAIR_IP ]]; then
	export ROBAIR_IP=localhost
fi
export PATH="$PATH:$ROBAIR_HOME/scripts/"
source /opt/ros/kinetic/setup.bash
source $ROBAIR_HOME/catkin_ws/devel/setup.bash
