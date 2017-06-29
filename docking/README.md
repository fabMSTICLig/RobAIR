# Docking

## Description

RobAIR possède maintenant une capacitée de docking. Il peut se docker à la base de chargement si celle-ci est à proximité. Deux solutions ont été créé :

- Docking actif : Un RaspberryPi et sa caméra son installé sur la base de chargement. La caméra permet de détecter le marqueur prédisposé sur RobAIR. RobAIR et la base font parti du même réseau ROS pour communiquer.

- Docking passif : Le RaspberryPi de RobAIR est équipé d'une caméra. Elle permet de repérer le marqueur prédisposé sur la base pour se docker.

## Installation Docking Actif

### Prérequis

Vous aurez besoin d'un RaspberryPi supplémentaire pour la base.
Installer dessus :

- La dernière version de [ROS pour RaspberryPi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)
- La dernière version de [OpenCV](http://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html)

###### Sur RobAIR :

Placer un marqueur OpenCV à l'arrière de RobAIR.

###### Sur la base :

Placer la caméra de la base verticalement, à hauteur du marqueur.

### Installation

###### Depuis le RaspberryPi de la base :

- Installer le paquet ROS `robairdock` sur le RaspberryPi de la base dans `RobAIR/catkin_ws/src/`.

- Adapter les constantes à votre RobAIR en modifiant le fichier `dockmain.py` qui se trouve dans `RobAIR/catkin_ws/src/robairdock/scripts/`.

- Rendez-vous dans le répertoire `RobAIR/catkin_ws/` pour compiler et sourcer les paquets :
```bash
$ catkin_make
$ . devel/setup.bash
```

- Ajouter à la fin du fichier `.bashrc` les lignes suivantes :
```bash
export ROS_IP = [adresse ip de la base]
export ROS_MASTER_URI = http://[adresse ip de RobAIR]:11311
rosrun robairdock dockmain.py
```

- Redémarrer le RaspberryPi.

###### Depuis le RaspberryPi de RobAIR :**

- Adapter les constantes à votre RobAIR en modifiant le fichier `dock_active.py` qui se trouve dans `RobAIR/catkin_ws/src/robairmain/scripts/`.

- Executer la commande suivante pour passer en mode actif :
```
$ robair dockmode active
```

- Redémarrer RobAIR :
```
$ robair restart
```

## Installation Docking Passif

### Prérequis

Installer sur le RaspberryPi de RobAIR :
-  La dernière version de [OpenCV](http://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html)

### Installation

**Depuis le RaspberryPi de RobAIR :**

- Vérifier que le paquet ROS `robairmain` est bien installé dans `RobAIR/catkin_ws/src/`

- Adapter les constantes à votre RobAIR en modifiant le fichier `dock_passive.py` qui se trouve dans `RobAIR/catkin_ws/src/robairmain/scripts/`.

- Executer la commande suivante pour passer en mode passif :
```
$ robair dockmode passive
```

- Redémarrer RobAIR :
```
$ robair restart
```

*Note : Le paquet ROS `robairdock` n'est pas utilisé.*

## Utilisation

### Sous ROS

Pour docker RobAIR depuis le réseau ROS, il suffit de publier un `1` sur le topic `dockstate`. Si RobAIR est incapable de se docker, il repassera en état `0` de lui même.
La procédure de docking peut-être annulée à tout moment en publiant un `0` sur ce même topic.

#### Les états

- 0 = Non amarré
- 1 = Demande d'amarrage
- 2 = Non vu
- 3 = Vu
- 4 = Amarré

### Sous OpenHab

Créer l'item `ROBAIR_Start_Docking` de type `SwitchItem`.

Pour docker RobAIR depuis le réseau OpenHab, il suffit de passer l'item `ROBAIR_Start_Docking` à `ON`.
La procédure de docking peut-être annulée à tout moment en passant ce même item à `OFF`.

## Marqueurs OpenCV

Vous trouverez dans le répertoire `RobAIR/docking/Marker/` quelques marqueurs utilisés pour le docking.
Vous pouvez facilement générer des marqueurs avec le script `Aruco Marker Creator.py` et tester la détection de marqueur avec le script `Aruco Detection.py` qui se trouvent dans `RobAIR/docking/`.

## Calibration de la caméra

**Si vous utilisez une caméra RaspberryPi v1.3 ou v2.1**

La calibration de votre caméra à déjà été faite. Vous pouvez récupérer les matrices dans `RobAIR/docking/Measurements/`.
Les images utilisées pour la calibration se trouve dans le dossier `RobAIR/docking/Calibration_Images/`.

**Si la calibration n'a pas été faite pour votre caméra**

- Utilisez le script `Calibration_Image_Creator.py` qui se trouve dans `RobAIR/docking/` pour créer les images nécéssaires à la calibration.

- Dans votre dossier opencv, utilisez le fichier `calibrate.py` qui se trouve dans `sample/python/` pour calibrer la caméra. N'oubliez pas de renseigner ligne 38 le chemin des images.
