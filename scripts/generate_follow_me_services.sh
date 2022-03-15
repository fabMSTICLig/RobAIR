#!/bin/bash

### Check et installation des packages necessaires pour GPIO

if [ "`which pip3`" == "" ] ; then 
	sudo apt install -y python3-pip 
	if [ $? -ne 0 ]; then 
		echo -e "unable to locate or install pip3\nAbort.."
    exit 1
	fi
fi
if [ "`pip3 list | grep 'RPi\.GPIO'`" == "" ] ; then 
	sudo apt install -y python3-dev python3-rpi.gpio python3-gpiozero
	if [ $? -ne 0 ]; then 
		echo -e "unable to locate or install python3 module: RPi.GPIO\nAbort.."
    exit 1
	fi
fi
python3 -c "import gpiozero" 2>/dev/null
if [ $? -ne 0 ] ; then
	sudo pip3 install gpiozero
	if [ $? -ne 0 ] ; then
		echo -e "unable to locate or install python3 package: gpiozero\nAbort.."
		exit 1
	fi
fi
python3 -c "import rpi_ws281x" 2>/dev/null
if [ $? -ne 0 ] ; then
	sudo pip3 install rpi_ws281x
	if [ $? -ne 0 ] ; then
		echo -e "unable to locate or install python3 package: rpi_ws281x\nAbort.."
		exit 1
	fi
fi
python3 -c "import adafruit-circuitpython-neopixel" 2>/dev/null
if [ $? -ne 0 ] ; then
	sudo pip3 install adafruit-circuitpython-neopixel
	if [ $? -ne 0 ] ; then
		echo -e "unable to locate or install python3 package: adafruit-circuitpython-neopixel\nAbort.."
		exit 1
	fi
fi

# Creations des services Follow_me et Switch Mode RobAIR

services='follow_me_robair switch_robair_mode'

if [ -z $ROBAIR_HOME ]; then
	echo -e "\033[31mROBAIR_HOME not define\033[0\ntry to:\n\texport ROBAIR_HOME=/home/${USER}/RobAIR\nAbort..."
fi

for service in $services; do
    sed "s#<ROBAIR_HOME>#$ROBAIR_HOME#g" \
        "${ROBAIR_HOME}/configs/${service}.template" \
        > "${ROBAIR_HOME}/configs/${service}.service"
    systemctl --user link "${ROBAIR_HOME}/configs/${service}.service"
    if [ $? -ne 0 ]; then
	    echo -e '\033[31merror in service linking\033[0\nAbort...'
	    exit 1
    fi
	echo -e "\033[32mservice '$service' linked\033[0m"
done

# Enable switch_robair_mode
    systemctl --user enable "${ROBAIR_HOME}/configs/switch_robair_mode.service"
    if [ $? -ne 0 ]; then
	    echo -e '\033[31merror in service enabling\033[0\nAbort...'
	    exit 1
    fi

for service in $services; do
    echo -n "Service: $service : "
    systemctl --user is-active $service 
done

# creation du service autologin pour que le user se log au boot et start les services RobAIR

sudo mkdir -p /etc/systemd/system/getty@tty1.service.d/
sudo echo "[Service]
ExecStart=
ExecStart=-/sbin/agetty --noissue --autologin $USER %I \$TERM
Type=idle" | sudo tee /etc/systemd/system/getty@tty1.service.d/override.conf
