#!/bin/bash

if [ -z $ROBAIR_HOME ]; then
	echo "La variable ROBAIR_HOME doit être définie."
	exit
fi

PORT=`ls /dev/serial/by-id/`
if [ -z $PORT ]; then
	echo "Aucune carte arduino connectée"
else
	sed -i -e 's#".*Arduino[^"]*"#"port" value="/dev/serial/by-id/'"$PORT"'"#g' $ROBAIR_HOME/catkin_ws/src/robairmain/launch/robair.launch
fi
