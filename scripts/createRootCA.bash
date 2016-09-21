#!/bin/bash

if [ -z $ROBAIR_HOME ]; then
	echo "La variable ROBAIR_HOME doit être définie."
	exit
fi

openssl genrsa -out $ROBAIR_HOME/ssl/rootCA.key 2048
echo "$(tput setaf 1)Clé root créée"
echo "$(tput setaf 1)Veuillez remplir les champs qui vous sont demandés"
openssl req -x509 -new -nodes -key $ROBAIR_HOME/ssl/rootCA.key -sha256 -days 1024 -out $ROBAIR_HOME/ssl/rootCA.crt
echo "$(tput setaf 1)Autorité de certification créée $ROBAIR_HOME/ssl/rootCA.crt"
echo "$(tput setaf 1)Veillez installer ce fichier sur chaque ordinateur qui pilotera un Robair. Celui-ci y compris"
echo "$(tput setaf 1)sudo cp $ROBAIR_HOME/ssl/rootCA.crt /usr/local/share/ca-certificates/"
echo "$(tput setaf 1)sudo update-ca-certificates"
