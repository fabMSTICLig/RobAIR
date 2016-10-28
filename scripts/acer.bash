#!/usr/bin/env bash


alias "chpro='chromium-browser --proxy-server=\"http://www-cache.ujf-grenoble.fr:3128\"'" >> .bash_aliases
echo "alias setproxy='export http_proxy=http://www-cache.ujf-grenoble.fr:3128'" >> .bash_aliases
source .bashrc

setproxy

sudo apt-get update
sudo apt-get install git vim chromium-browser gvfs-bin uncrustify

wget https://github.com/atom/atom/releases/download/v1.11.2/atom-amd64.deb
sudo dpkg --install atom-amd64.deb

git clone git@github.com:FabmsticLig/Rob-AIR.git
cd Rob-AIR
./scripts/install.bash
