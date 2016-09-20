#!/bin/bash

if [ -z $ROBAIR_HOME ]; then
	echo "La variable ROBAIR_HOME doit être définie."
	echo "Veuillez ajouter les deux lignes suivante à votre ~/.bashrc"
	echo "export ROBAIR_HOME=~/robair"
	echo "source \$ROBAIR_HOME/scripts/env.bash"
	exit
fi

sudo apt-get update
sudo apt-get install coturn

git clone https://github.com/andyet/signalmaster.git
