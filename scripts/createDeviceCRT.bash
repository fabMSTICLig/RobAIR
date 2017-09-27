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

v3_conf=$(mktemp)
cat > "$v3_conf" <<EOF
authorityKeyIdentifier=keyid,issuer
basicConstraints=CA:FALSE
keyUsage = digitalSignature, nonRepudiation, keyEncipherment, dataEncipherment
subjectAltName = @alt_names

[alt_names]
IP.1 = $ROBAIR_IP
EOF

openssl genrsa -out $ROBAIR_HOME/ssl/device.key 2048
openssl req -new -key $ROBAIR_HOME/ssl/device.key -out $ROBAIR_HOME/ssl/device.csr -subj '/C=FR/ST=Is\xC3\x83\xC2\xA8re/L=Grenoble/O=LIG/OU=FabMSTIC/CN='"$ROBAIR_IP"'/emailAddress=fabmstic.lig@gmail.com'
openssl x509 -req -in $ROBAIR_HOME/ssl/device.csr -CA $ROBAIR_HOME/ssl/rootCA.crt -CAkey $ROBAIR_HOME/ssl/rootCA.key -CAcreateserial -out $ROBAIR_HOME/ssl/device.crt -days 500 -sha256 -extfile $v3_conf

rm -f "$v3_conf"
