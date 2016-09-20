#!/bin/bash
if [ -z $ROBAIR_IP ]; then
	echo "La variable ROBAIR_IP doit être définie"
	exit
fi
turnserver -L $ROBAIR_IP  -a -f -v -c /home/robair/robair/configs/turnserver.conf -r $ROBAIR_IP
