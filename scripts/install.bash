#!/bin/bash

################################################################################
# Tests d'intégrité, variables d'environnement, setup pour l'install           #
################################################################################

# Tests

if [ ! -f .robair ]; then
	echo "Vous devez être dans le répertoire root du dépot"
	echo "Et exécuter scripts/install.sh"
	exit 1
fi


# Variables

export ROBAIR_HOME=`pwd`


# Mise en place du logging et de l'affichage

source "$ROBAIR_HOME/scripts/helpers.sh"
start_quiet_mode install.log


################################################################################
# Questions à l'utilisateur                                                    #
################################################################################

# Fonctions d'aide

ask() {
	typeset -n conf_var=$1
	read -r -p "$2" conf_var 1>&3 2>&4
	unset -n conf_var
}

ask_yn() {
	typeset -n conf_var=$1

	res=x
	while [ "$res" = 'x' ]; do
		read -r -p "$2" res 1>&3 2>&4
		case "$res" in
			[OoYy])
				res=y
				;;
			[Nn])
				res=n
				;;
			'')
				res=$conf_var
				;;
			*)
				res=x
				;;
		esac
	done

	conf_var=$res
	unset -n conf_var
}


echo "$(tput setab 2)Configuration$(tput sgr0)" >&3

# Bashrc

if grep ROBAIR ~/.bashrc >>/dev/null 2>&1; then
	do_bashrc=n
else
	do_bashrc=y
	ask_yn do_bashrc "Faut-il configurer votre ~/.bashrc ? [O/n] "
fi


# Proxies

if [ -z "$http_proxy" ]; then
	ask http_proxy "Si vous avez un proxy, veuillez l'indiquer. Sinon laissez vide. [] "
	if [ -n "$http_proxy" ]; then
		same_proxy=y
		ask_yn same_proxy "Votre proxy HTTPS est-il identique ? [O/n] "
		[ "$same_proxy" = 'y' ] && https_proxy="$http_proxy"
	fi
fi
export http_proxy
export https_proxy


# Lancement au démarrage par systemd

do_enable_systemd_unit=n
if which systemctl >>/dev/null 2>&1; then
	do_enable_systemd_unit=y
	ask_yn do_enable_systemd_unit "Voulez-vous lancer RobAIR au démarrage ? [O/n] "
fi



##########################################
# Configuration du ~/bashrc
# et création des variables d'environement correspondantes
##########################################

echo >&3
echo "$(tput setab 2)Installation$(tput sgr0)" >&3

# Demande le mot de passe pour les prochaines invocations de sudo
sudo true
if [ "$?" -ne 0 ]; then
	echo "[$(tput setaf 1)ERREUR$(tput sgr0)]" \
		"Vous devez avoir les droits superutilisateur pour installer RobAIR" >&4
	exit 1
fi

# Bashrc
if [ "$do_bashrc" = 'y' ]; then
	start_job "Configure le bashrc"
	echo "" >> ~/.bashrc
	echo "#ROBAIR SETTINGS" >> ~/.bashrc
	echo "export ROBAIR_HOME=$ROBAIR_HOME" >> ~/.bashrc
	echo "source \$ROBAIR_HOME/scripts/env.bash"  >> ~/.bashrc
	end_job
fi

export PATH="$PATH:$ROBAIR_HOME/scripts/"

# Droits d'accès au port série
if ! groups | grep dialout &>/dev/null; then
	start_job "Donne à l'utilisateur l'accès aux ports série"
	sudo adduser "$USER" dialout
	end_job
fi

# Ajoute une règle pour que ModemManager ne considère pas les Arduino pour sa
# recherche de modems (évite que le périphérique soit temporairement utilisé au
# démarrage)
if [ ! -f /etc/udev/rules.d/77-arduino.rules ]; then
	start_job "Ajoute les règles udev pour Arduino"
	sudo sh -c "echo 'ATTRS{idVendor}==\"2a03\", ENV{ID_MM_DEVICE_IGNORE}=\"1\"' > /etc/udev/rules.d/77-arduino.rules"
	end_job
fi

# Installation des paquets
start_job "Ajoute les dépôts ROS"
sudo sh -c 'echo "deb [signed-by=/usr/share/keyrings/ros.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'curl https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor > /usr/share/keyrings/ros.gpg'
end_job

start_job "Met à jour les listes de paquets"
sudo -E apt-get update
end_job


start_job "Installe les paquets nécessaires"
sudo -E apt-get -y install arduino-core \
	ros-noetic-ros-base ros-noetic-urg-node \
	ros-noetic-tf
sudo -E apt-get -y install python3-rosdep python3-rosinstall \
       	python3-rosinstall-generator python3-wstool build-essential
end_job

start_job "Met à jour les dépendances ROS"
source /opt/ros/noetic/setup.bash
sudo -E rosdep init
rosdep update
end_job

# Récupère les sous-modules
start_job "Récupère les dépendances du dépôt Git"
git submodule update --init
end_job

# Compile les packages ROS
start_job "Compile les paquets ROS locaux"
(cd "$ROBAIR_HOME/catkin_ws" && catkin_make install)
end_job

source "$ROBAIR_HOME/catkin_ws/devel/setup.bash"

# Génère la ros_lib
start_job "Génère la bibliothèque ros_lib pour Arduino"
rm -rf "$ROBAIR_HOME/arduino/libraries/ros_lib"
rosrun rosserial_arduino make_libraries.py "$ROBAIR_HOME/arduino/libraries"
end_job

# Génère le service systemd et l'ajoute au système
start_job "Génère le service systemd"
sed "s#<ROBAIR_HOME>#$ROBAIR_HOME#g ; s#<UID>#$UID#g" \
	"$ROBAIR_HOME/configs/robair.service.template" \
	> "$ROBAIR_HOME/configs/robair.service"

if which systemctl >>/dev/null 2>&1; then
	if [ "$do_enable_systemd_unit" = 'n' ]; then
		systemctl --user link "$ROBAIR_HOME/configs/robair.service"
	else
		systemctl --user enable "$ROBAIR_HOME/configs/robair-wait-online.service"
		systemctl --user enable "$ROBAIR_HOME/configs/robair.service"
	fi
fi
end_job

echo >&3
echo "$(tput setab 2)Installation terminée$(tput sgr0)" >&3
