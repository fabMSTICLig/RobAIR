#!/bin/bash

if [ ! -f .robair ]; then
	echo "Vous devez être dans le répertoire root du dépot"
	echo "Et exécuter scripts/install.sh"
	exit
fi

export ROBAIR_HOME=`pwd`

##########################################
# Configuration du ~/bashrc
# et création des variables d'environement correspondantes
##########################################

testbash=`cat ~/.bashrc | grep ROBAIR`
if [[ -z $testbash ]]; then
	read -r -p "Configurer ~/bashrc ? [O/n] "
	if [[ $REPLY  =~ ^[Oo]$ ||  $REPLY =~ ^$ ]]; then
		echo "" >> ~/.bashrc
		echo "#ROBAIR SETTINGS" >> ~/.bashrc
		echo "export ROBAIR_HOME=$ROBAIR_HOME" >> ~/.bashrc
		echo "source \$ROBAIR_HOME/scripts/env.bash"  >> ~/.bashrc
	fi
fi

export PATH="$PATH:$ROBAIR_HOME/scripts/"

if [ -z $http_proxy ]; then
	read -r -p "Veuillez entrer votre proxy ou tapez entrée si vous n'en avez pas:" response
	if [ ! -z "$response" ]; then
		export http_proxy=$response
	fi
fi

if [ -z $https_proxy] && [ ! -z $http_proxy ]; then
	read -r -p "Votre proxy https est identique au proxy http ? [O/n] "
	if [[ $REPLY  =~ ^[Oo]$ ||  $REPLY =~ ^$ ]]; then
        	export https_proxy=$http_proxy
	fi
fi


echo "Veuillez entrer votre mot de passe pour installer les packages (sudo)"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo -E apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo -E apt-get update


echo "$(tput setaf 1)Installation $(tput setab 7)coturn nodejs ros-kinetic arduino$(tput sgr0)"
sudo -E apt-get install coturn nodejs-legacy npm chromium-browser \
	ros-kinetic-ros-base ros-kinetic-rosbridge-suite arduino

source /opt/ros/kinetic/setup.bash
sudo -E rosdep init
rosdep update

read -r -p "Voulez-vous générer une autorité de certification ? [O/n] " response
case $response in
	[nN])
		read -r -p "Copier les fichiers rootCA.crt rootCA.key dans $ROBAIR_HOME/ssl puis appuyer sur entrer " response
		;;
 	*)
		./scripts/createRootCA.bash
		;;
esac


read -r -p "Voulez-vous générer un certificat ssl ? [O/n] "
if [[ $REPLY  =~ ^[Oo]$ ||  $REPLY =~ ^$ ]]; then
	./scripts/createDeviceCRT.bash
fi


# Récupère les sous-modules
git submodule update --init

# Compile les packages ROS
(cd "$ROBAIR_HOME/catkin_ws" && catkin_make install)
source "$ROBAIR_HOME/catkin_ws/devel/setup.bash"

# Récupère les dépendances pour l'interface web
(cd $ROBAIR_HOME/interface && npm install)

# Configure signalmaster
(cd "$ROBAIR_HOME/signalmaster" && npm install)

# Génère la ros_lib
echo "$(tput setaf 1)Genère $ROBAIR_HOME/arduino/libraries/ros_lib$(tput sgr0)"
rm -rf "$ROBAIR_HOME/arduino/libraries/ros_lib"
rosrun rosserial_arduino make_libraries.py "$ROBAIR_HOME/arduino/libraries"

echo "$(tput setaf 1)Installation terminée$(tput sgr0)"
