#!/bin/bash

if [ -z $ROBAIR_HOME ]; then
	echo "La variable ROBAIR_HOME doit être définie."
	exit
fi


if [ ! -f "$ROBAIR_HOME/ssl/rootCA.crt" ]; then
	echo "le fichier $ROBAIR_HOME/ssl/rootCA.crt n'existe pas"
	echo "Générez le grâce à $ROBAIR_HOME/scripts/createRootCA.bash"
	echo "Ou importé le dans le dossier $ROBAIR_HOME/ssl/"
	exit
fi

if [ ! -f "$ROBAIR_HOME/ssl/rootCA.key" ]; then
	echo "le fichier $ROBAIR_HOME/ssl/rootCA.key n'existe pas"
	echo "Générez le grâce à $ROBAIR_HOME/scripts/createRootCA.bash"
	echo "Ou importé le dans le dossier $ROBAIR_HOME/ssl/"
	exit
fi

openssl genrsa -out $ROBAIR_HOME/ssl/device.key 2048
echo "===================================================================================="
echo "Dans le champs Common Name veuillez indiquer votre adresse IP $ROBAIR_IP"
echo "===================================================================================="
openssl req -new -key $ROBAIR_HOME/ssl/device.key -out $ROBAIR_HOME/ssl/device.csr
openssl x509 -req -in $ROBAIR_HOME/ssl/device.csr -CA $ROBAIR_HOME/ssl/rootCA.crt -CAkey $ROBAIR_HOME/ssl/rootCA.key -CAcreateserial -out $ROBAIR_HOME/ssl/device.crt -days 500 -sha256

