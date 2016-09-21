# Rob-AIR

#INSTALL


```bash
git clone https://github.com/FabmsticLig/Rob-AIR.git
cd Rob-AIR
```

Exécutez les deux ligne suivantes et ajoutez les à votre .bashrc 

ROBAIR_HOME doit corespondre au dossier ou vous avez cloné le dépôt
```bash
export ROBAIR_HOME=~/Rob-Air
source $ROBAIR_HOME/scripts/env.bash
```

Installation des logiciel et librairies tiers

```bash
install.sh
```

SSL/TLS
Création de l'autorité de certification

Si vous avez déjà généré une autorité de certification sur un autre ordinateur par exemple, veuillez copier les fichiers rootCA.crt  rootCA.key  rootCA.srl dans $ROBAIR_HOME/ssl

Sinon exécutez :

```bash
createRootCA.sh
```

Création du certificat du Robair, à exécuter à chaque changement d'addresse IP
```bash
createDeviceCRT.sh
```

Configure les fichiers de configurations

```bash
configure.sh
```



