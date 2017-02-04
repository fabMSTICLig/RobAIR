# Rob-AIR

##INSTALL

### Prérequis

OS : Ubuntu 16.04 LTS

Vous devez préalablement installer l'utilitaire `git`.

###Installation

Clone du repository
```bash
$ git clone https://github.com/FabmsticLig/RobAIR.git
```

Installation par le script
```bash
$ cd RobAIR
$ ./scripts/install.bash
```

Le script utilise sudo pour installer plusieurs packages. Il faudra rentrer votre mot de passe utilisateur.

Le script rajoute deux variable d'environnement à votre `.bashrc`. `ROBAIR_HOME` qui correspond au dossier racine de ce repository. `ROBAIR_IP` l'IPV4 de l'ordinateur utilisé pour configurer les différents programmes de Robair. Le script `$ROBAIR_HOME/scripts/env.bash` est sourcé.

A la fin de l'installation veuillez exécuter `source ~/.bashrc`afin de prendre en compte ces modifications.

##CONFIGURATION
Différents scripts permettent de faciliter l'utilisation de Robair.

### Changement d'IP

Lorsque Robair change de réseau, son IP change. Il est nécessaire de mettre à jour différents fichiers de configuration. Ceci est effectué par le script `changeip.bash`. Il met à jour les fichiers de configuration de l'interface web et génère le certificat SSL correspondant. 


###SSL/TLS
Robair est piloté par une interface web. Afin de sécuriser la connexion et d'être compatible avec les navigateurs, les différentes connexions utilisent un certificat SSL.

Le certificat dois être accepté par une autorité de certification. Ces autorités de certification sont généralement payantes. La solution adotpée est de créer soi-même cette autorité. Cette autorité devra être importée dans le ou les navigateur(s) pilotant RobAIR.
A partir de cette autorité, vous pouvez générer des certificats SSL qui seront accepté par votre navigateur.
Une autorité peut donc être utilisé pour générer les certificats de plusieurs RobAIR.

####Création de l'autorité de certification

Si vous avez déjà généré une autorité de certification sur un autre ordinateur par exemple, veuillez copier les fichiers rootCA.crt  rootCA.key  rootCA.srl dans `$ROBAIR_HOME/ssl`

Sinon exécutez :

```bash
$ createRootCA.sh
```

####Création du certificat du Robair
Une fois l'autorité de certificat créé, il faut créer le certificat de RobAIR. Cette opération est à effectuer à chaque changement d'addresse IP pour plus de facilité le script `changeip.bash` peut également être utilisé .
```bash
$ createDeviceCRT.sh
```

###Changement carte arduino
Le script `changearduino.bash` permet de configurer le fichier `robair.launch` afin que ROS se connecte à la bonne carte Arduino.
Une seule carte disposant d'une laison serie doit être connecté à l'ordinateur avant de lancer ce script. 
Ce script n'est utile que la carte arduino est changée.

##Utilisation

###Première utilisation

- Allumez RobAIR
- Branchez la tablette
- Allumer la tablette
- Connectez vous au réseau wifi. Si votre RobAIR dispose d'une carte wifi externe pensez à déconnecté la carte wifi interne (INTEL).
- Ouvrez un terminal et lancez la commande `changeip.bash`, notez l'adresse IP indiqué (ROBAIR_IP).
- Lancer la commande `robair start`
- Sur la machine utilisateur, allez à l'adresse http://ROBAIR_IP:6081 (remplacez ROBAIR_IP par l'actuelle adresse IP)
- Récupérez l'autorité de certification et installez la sur le navigateur utilisateur.
- Allez à l'adresse https://ROBAIR_IP:6080


###Environnement RobAIR
Le script `robair` vous permet de lancer, arreter ou relancer les différents programme de RobAIR.
Le programme sous-jacent utiliser est l'utilitaire `roslaunch` de ROS

```bash
$ robair start
```
La commande  `robair start` lance les différents programmes de RobAIR. Vous pouvez ensuite utilisé Ctrl+C pour l'arrêter. 
```bash
$ robair stop
```
La commande  `robair stop` arrête l'ensemble des programmes. Utile quand vous n'avez plus acces au terminal qui a lancer le programme. 
```bash
$ robair restart
```
La commande  `robair restart` arrête l'ensemble des programmes puis les relance après 3 secondes. 
