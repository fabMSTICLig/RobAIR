<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/3.0/fr/"><img alt="Licence Creative Commons" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/3.0/fr/88x31.png" /></a><br />Les différentes partie de ce projet sont mise à disposition selon les termes de la <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/3.0/fr/">Licence Creative Commons Attribution - Pas d’Utilisation Commerciale - Partage dans les Mêmes Conditions 3.0 France</a>.

---

Avec le soutien financier de ![la région Auvergne-Rhône-Alpes](https://www.auvergnerhonealpes.fr/images/GBI_CRRAA/logo.png)

---

# RobAIR

## Installation

### Pré-requis

- Ubuntu 20.04 LTS
- Git

- L'utilisateur courant doit avoir les droits root via `sudo`

### Procédure

- Téléchargez le dépôt
```bash
git clone https://github.com/FabmsticLig/RobAIR.git
```
- Lancez le script d'installation
```bash
cd RobAIR
./scripts/install.bash
```
- Si l'utilisateur courant n'était préalablement pas dans le groupe `dialout`
(autrement dit, si le script d'installation a affiché "donne à l'utilisateur
l'accès aux ports série"), redémarrez votre session
- Avec la carte Arduino branchée, lancez le chargement des programmes
```bash
robair reload
```

Le script d'installation récupérera des dépôts les paquets nécessaires et
effectuera la configuration initiale de RobAIR. Quelques variables seront aussi
ajoutées au bashrc de l'utilisateur, en plus d'un service systemd permettant de
démarrer et arrêter les programmes.


## Utilisation

### Procédure

Sur RobAIR
- Allumez le robot
- Allumez la tablette
- Branchez la tablette
- Connectez vous au réseau wifi. Si votre RobAIR dispose d'une carte wifi
externe pensez à déconnecter la carte wifi interne (INTEL).
- Ouvrez un terminal et lancez la commande `robair ip`, notez l'adresse IP indiquée (ROBAIR_IP)
- Lancez la commande `robair start`
- Ouvrez chromium et allez à l'adresse https://ROBAIR_IP:6080 (en remplaçant
ROBAIR_IP par l'adresse notée précédemment)

Sur la machine utilisateur
- Allez à l'adresse http://ROBAIR_IP:6081
- Récupérez l'autorité de certification et installez-la sur le navigateur client
- Allez à l'adresse https://ROBAIR_IP:6080


### Gestion

La partie logicielle de RobAIR peut être contrôlée depuis la tablette via le
programme `robair`. Utilisez les commandes suivantes pour respectivement lancer,
arrêter ou redémarrer le logiciel.
```bash
robair start
robair stop
robair restart
```

Si `systemd` est installé (par défaut sous Ubuntu), ces commandes pilotent
RobAIR via un service systemd en session utilisateur. L'intérêt est
essentiellement de permettre le lancement de RobAIR au démarrage, puis de gérer
automatiquement les erreurs (`systemd` relancera le logiciel s'il venait à
planter). Avec quelques modifications il serait aussi possible de
configurer l'activation par socket.

Le programme `robair` permet aussi la mise à jour du logiciel via cette commande
```bash
$ robair update
```
Cela récupérera la dernière version de RobAIR via Git, recompilera les
programmes et mettra à jour la carte Arduino.


## Développement

RobAIR est développé au-dessus de l'architecture [ROS][1].

L'essentiel du code du robot se trouve dans `arduino/libraries`, avec le point
d'entrée dans `arduino/robairarduino`.

L'interface est servie par un serveur `nodejs` résidant dans `interface`.

Si vous souhaitez tester des modifications locales, vous pouvez recompiler les
programmes et recharger la carte Arduino via cette commande
```bash
robair reload
```

 [1]: http://www.ros.org/
