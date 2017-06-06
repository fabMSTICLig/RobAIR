# Docking

## Description

RobAIR possède maintenant une capacitée de docking. Il peut se docker à la base de chargement si celle-ci est à proximité. Deux solutions ont été créé :

- Docking actif : Un RaspberryPi et sa caméra son installé sur la base de chargement. La caméra permet de détecter le marqueur prédisposé sur RobAIR. RobAIR et la base font parti du même réseau ROS pour communiquer.

- Docking passif : Le RaspberryPi de RobAIR est équipé d'une caméra. Elle permet de repérer le marqueur prédisposé sur la base pour se docker.

## Installation Docking Actif

### Prérequis

Vous aurez besoin d'un RaspberryPi supplémentaire pour la base. Il faut installer dessus :

- La dernière version de ROS : http://wiki.ros.org/kinetic/Installation/Ubuntu
- La dernière version de OpenCV avec Aruco sur le RaspberryPi de RobAIR

### Installation

Installer le paquet `robairdock` sur le RaspberryPi de la base.

Depuis le répertoire `catkin_ws` initialement créé :

Compiler les paquets :
```bash
$ catkin_make
```
Sourcer les paquets :
```bash
$ . devel/setup.bash
```

Pour lancer le noeud de manière autonome au démarage du RaspberryPi, il faut ajouter les lignes suivantes en fin du fichier `.bashrc` :
```bash
export ROS_IP= [adresse ip de la base]
export ROS_MASTER_URI=http://[adresse ip de RobAIR]:11311
rosrun robairdock dockmain.py
```

Dans le fichier `RobAIR/xxx/yyy/zzz` décommenter l'instruction permettant le lancement du noeud sous docking actif : `rosrun robairmain dock_active.py`. L'instruction lançant le noeud sous docking passif peut-être supprimée.

## Installation Docking Passif

### Prérequis

Installer la dernière version de OpenCV avec Aruco sur le RaspberryPi de RobAIR :

### Installation

Note : Le paquet `robairdock` n'est pas utilisé.

Dans le fichier `RobAIR/xxx/yyy/zzz` décommenter l'instruction permettant le lancement du noeud sous docking actif : `rosrun robairmain dock_passive.py`. L'instruction lançant le noeud sous docking actif peut-être supprimée.

## Utilisation

### Docking

Pour docker RobAIR indépendement de la solution choisie, il suffit de publier un `1` sur le topic `dockstate`. Si RobAIR est incapable de se docker, il repassera en état `0` de lui même.
La procédure de docking peut-être annulée à tout moment en publiant un `0` sur le topic `dockstate`.

### Les états

0 = Non amarré

1 = Demande d'amarrage

2 = Non vu

3 = Mal placé

4 = Amarrage en cours

5 = Amarré




