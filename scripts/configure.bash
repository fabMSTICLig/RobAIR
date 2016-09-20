#!/bin/bash
if [ -z $ROBAIR_HOME ]; then
	echo "La variable ROBAIR_HOME doit être définie."
	echo "Veuillez ajouter les deux lignes suivante à votre ~/.bashrc"
	echo "export ROBAIR_HOME=~/robair"
	echo "source \$ROBAIR_HOME/scripts/env.bash"
	exit
fi
cp $ROBAIR_HOME/configs/signalmaster.json $ROBAIR_HOME/signalmaster/config/development.json
python $ROBAIR_HOME/scripts/editjson.py $ROBAIR_HOME/signalmaster/config/development.json server:key $ROBAIR_HOME/ssl/device.key
python $ROBAIR_HOME/scripts/editjson.py $ROBAIR_HOME/signalmaster/config/development.json server:cert $ROBAIR_HOME/ssl/device.crt
