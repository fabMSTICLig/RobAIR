#!/bin/bash
#  remove old log file in ~/.ros/log/
#  must be place in ${ROBAIR_HOME}/scripts/.
# 
#  setup crontab to autoexec:
#  echo "@reboot    $USER    ${ROBAIR_HOME}/scripts/rm_old_log_ros.sh" | sudo tee --append /etc/crontab

LOG_PATH="$HOME/.ros/log"

for elem in  `ls -d ${LOG_PATH}/*/`
do 
	if [ "$elem" != "`ls -l ${LOG_PATH} | grep latest | tr -d " " | awk -F '>' '{print $NF}'`/"  ] && [ "$elem" != ${HOME}"/.ros/log/latest/" ]
then
	    rm -rf $elem
	fi
done


