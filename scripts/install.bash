#!/bin/bash -e

if [ ! -f .robair ]; then
	echo "Vous devez être dans le répertoire root du dépot"
	echo "Et exécuter scripts/install.sh"
fi

export ROBAIR_HOME=`pwd`

##########################################
# Configuration du ~/bashrc
# et création des variables d'environement correspondantes
##########################################

read -r -p "Configurer ~/bashrc ? [O/n] "
if [[ $REPLY  =~ ^[Oo]$ ||  $REPLY =~ ^$ ]]; then
	echo "" >> ~/.bashrc
	echo "#ROBAIR SETTINGS" >> ~/.bashrc
	echo "export ROBAIR_HOME=$ROBAIR_HOME" >> ~/.bashrc
	echo "source \$ROBAIR_HOME/scripts/env.bash"  >> ~/.bashrc
fi

# Récupère l'IP actuel du Robair
export ROBAIR_IP=`ip route get 8.8.8.8 | awk 'NR==1 {print $NF}'`
export PATH="$PATH:$ROBAIR_HOME/scripts/"


read -r -p "Veuillez entrer votre proxy ou tapez entrée si vous n'en avez pas:" response
if [ ! -z $reponse ]; then
	export http_proxy=$reponse
	git config --global http.proxy $http_proxy
fi

echo "Veuillez entrer votre mot de passe pour installer les packages (sudo)"

sudo -E apt-get update




echo "$(tput setaf 1)Installation $(tput setab 7)coturn nodejs npm $(tput sgr0)"
sudo -E apt-get install git make coturn



read -r -p "Installation signalmaster ? [O/n] "
if [[ $REPLY  =~ ^[Oo]$ ||  $REPLY =~ ^$ ]]; then
	git clone https://github.com/andyet/signalmaster.git
	#Copie et edition de la configuration de signalmaster
	cp $ROBAIR_HOME/configs/signalmaster.json $ROBAIR_HOME/signalmaster/config/development.json
	python $ROBAIR_HOME/scripts/editjson.py $ROBAIR_HOME/signalmaster/config/development.json server:key $ROBAIR_HOME/ssl/device.key
	python $ROBAIR_HOME/scripts/editjson.py $ROBAIR_HOME/signalmaster/config/development.json server:cert $ROBAIR_HOME/ssl/device.crt
	
fi

read -r -p "Voulez vous générer une autorité de certification ? [O/n] " response
case $response in
	[nN]) 
		read -r -p "Copier les fichiers rootCA.crt rootCA.key dans $ROBAIR_HOME/ssl puis appuyer sur entrer " response
		;;
 	*)
		./scripts/createRootCA.bash
		;;
esac


read -r -p "Voulez vous générer un certificat ssl ? [O/n] "
if [[ $REPLY  =~ ^[Oo]$ ||  $REPLY =~ ^$ ]]; then
	./scripts/createDeviceCRT.bash
fi


read -r -p "Installation ros kinetic ? [O/n] "
if [[ $REPLY  =~ ^[Oo]$ ||  $REPLY =~ ^$ ]]; then

	echo "$(tput setaf 1)Installation $(tput setab 7)ros kinetic$(tput sgr0)"

	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
	sudo -E apt-get update
	sudo apt-get install ros-kinetic-ros-base
	source /opt/ros/kinetic/setup.bash
	sudo rosdep init
	rosdep update
	sudo apt-get install ros-kinetic-rosbridge-suite

fi




read -r -p "Installation rosserial ? [O/n] "
if [[ $REPLY  =~ ^[Oo]$ ||  $REPLY =~ ^$ ]]; then


	sudo apt-get install ros-kinetic-rosserial-arduino
	sudo apt-get install ros-kinetic-rosserial
	cd $ROBAIR_HOME/catkin_ws/src
	git clone https://github.com/ros-drivers/rosserial.git
	cd $ROBAIR_HOME/catkin_ws
	catkin_make
	catkin_make install
	source $ROBAIR_HOME/catkin_ws/devel/setup.bash

fi

echo "$(tput setaf 1)Installation $(tput setab 7) Arduino$(tput sgr0)"

sudo apt-get install arduino
echo "$(tput setaf 1)Arduino va être lancer veuillez accepter de rajouté dialout en rentrant votre mot de passe, puis fermer la fenêtre de Arduino$(tput sgr0)"
arduino
sed -i -e 's#\(.*sketchbook.path=\).*#\1'"$ROBAIR_HOME/arduino"'#' ~/.arduino/preferences.txt
echo "$(tput setaf 1)Genère $ROBAIR_HOME/arduino/libraries/ros_lib$(tput sgr0)"
mkdir -p $ROBAIR_HOME/arduino/libraries
cd $ROBAIR_HOME/arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .



echo "$(tput setaf 1)Instalation terminé$(tput sgr0)"
