# Rob-AIR

##INSTALL

### Prérequis

OS : Ubuntu 16.04 LTS

Vous devez préalablement installer l'utilitaire `git`.

###Installation

Clone du repository
```bash
git clone https://github.com/FabmsticLig/Rob-AIR.git
```

Installation par le script
```bash
cd Rob-AIR
./scripts/install.bash
```

Le script utilise sudo pour installer plusieurs packages. Il faudra rentrer votre mot de passe utilisateur.

Le script rajoute deux variable d'environnement à votre `.bashrc`. `ROBAIR_HOME` qui correspond au dossier racine de ce repository. `ROBAIR_IP` l'IPV4 de l'ordinateur utilisé pour configurer les différents programmes de Robair. Le script `$ROBAIR_HOME/scripts/env.bash` est sourcé.

A la fin de l'installation veuillez exécuter `source ~/.bashrc`afin de prendre en compte ces modifications.

##SCRIPTS
Différents scripts permettent de faciliter l'utilisation de Robair.

### Changement d'IP

Lorsque Robair change de réseau, son IP change. Il est nécessaire de mettre à jour différents fichiers de configuration. Ceci est effectué par le script `changeip.bash`. Il met à jour les fichiers de configuration de l'interface web et génère le certificat SSL correspondant. 


###SSL/TLS
Robair est piloté par une interface web. Afin de sécuriser la connexion et d'être compatible avec les navigateurs, les différentes connexions utilisent un certificat SSL.

Le certificat dois être accepté par une autorité de certification. Ces autorités de certification sont généralement payantes. La solution adotpée est de créer soient même cette autorité. Cette autorité devra être importée dans le ou les navigateur(s) pilotant Robair.
A partir de cette autorité, vous pouvez générer des certificats SSL qui seront accepté par votre navigateur.
Une autorité peut donc être utilisé par plusieurs Robair.

####Création de l'autorité de certification

Si vous avez déjà généré une autorité de certification sur un autre ordinateur par exemple, veuillez copier les fichiers rootCA.crt  rootCA.key  rootCA.srl dans $ROBAIR_HOME/ssl

Sinon exécutez :

```bash
createRootCA.sh
```

Création du certificat du Robair, à exécuter à chaque changement d'addresse IP (inclut dans le script `changeip.bash`)
```bash
createDeviceCRT.sh
```



