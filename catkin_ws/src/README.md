# Docking

## Description

RobAIR possède maintenant une capacitée de docking. Il peut se docker à la base de chargement si celle-ci est à proximité. Deux solutions ont été créé :

- Docking actif : Un RaspberryPi et sa caméra son installé sur la base de chargement. La caméra permet de détecter le marqueur prédisposé sur RobAIR. RobAIR et la base font parti du même réseau ROS pour communiquer.

- Docking passif : Le RaspberryPi de RobAIR est équipé d'une caméra. Elle permet de repérer le marqueur prédisposé sur la base pour se docker.

## Installation Docking Actif

### Prérequis

Vous aurez besoin d'un RaspberryPi supplémentaire pour la base.
Installer dessus :

- La dernière version de ROS : http://wiki.ros.org/kinetic/Installation/Ubuntu
- La dernière version de OpenCV avec Aruco sur le RaspberryPi de RobAIR : http://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html

###### Sur RobAIR :

Placer un marqueur OpenCV à l'arrière de RobAIR.

###### Sur la base :

Placer la caméra de la base verticalement, à hauteur du marqueur.

### Installation

###### Depuis le RaspberryPi de la base :

- Installer le paquet ROS `robairdock` sur le RaspberryPi de la base dans `catkin_ws/src/`.

- Rendez-vous dans le répertoire `catkin_ws` puis compiler les paquets :
```bash
$ catkin_make
```

- Sourcer les paquets :
```bash
$ . devel/setup.bash
```

- Ajouter à la fin du fichier `.bashrc` de la ligne suivante :
```bash
export ROS_IP= [adresse ip de la base]
export ROS_MASTER_URI=http://[adresse ip de RobAIR]:11311
rosrun robairdock dockmain.py
```

###### Depuis le RaspberryPi de RobAIR :**

- Ajouter à la fin du fichier `.bashrc` de RobAIR la ligne suivante :
```bash
rosrun robairmain dock_active.py
```

## Installation Docking Passif

### Prérequis

- Installer la dernière version de OpenCV avec Aruco sur le RaspberryPi de RobAIR : http://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html

### Installation

**Depuis le RaspberryPi de RobAIR :**

- Vérifier que le paquet ROS `robairmain` est bien installé dans `catkin_ws/src/`

- Ajouter à la fin du fichier `.bashrc` de RobAIR la ligne suivante :
```bash
rosrun robairmain dock_passive.py
```

*Note : Le paquet ROS `robairdock` n'est pas utilisé.*

## Utilisation

### Docking

Pour docker RobAIR indépendement de la solution choisie, il suffit de publier un `1` sur le topic `dockstate`. Si RobAIR est incapable de se docker, il repassera en état `0` de lui même.
La procédure de docking peut-être annulée à tout moment en publiant un `0` sur le topic `dockstate`.

### Les états

- 0 = Non amarré
- 1 = Demande d'amarrage
- 2 = Non vu
- 3 = Mal placé
- 4 = Amarrage en cours
- 5 = Amarré

## Marqueurs OpenCV

Vous trouverez dans le répertoire `docs/dock/Marker/` quelques marqueurs utilisés pour le docking.
Vous pouvez facilement générer des marqueurs avec le script `Aruco Marker Creator` qui se trouve dans `docs/dock/`.


