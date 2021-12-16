#!/bin/bash

# network manager doit etre installe avant
which NetworkManager >/dev/null

if [ $? -ne 0 ] ; then
	sudo apt install -y network-manager
	if [ $? -ne 0 ] ; then
		echo -e "network manager must be install\nAbort..."
		exit 1
	fi
fi

# desactive la configuration netplan actuelle

for file in `ls /etc/netplan/*.yaml` ; do
	sudo mv $file $file.bak ;
done

# NETPLAN config WIFI HotSpot
template_netplan_file="${ROBAIR_HOME}/configs/netplan_wifi.yaml.template"
netplan_file="/etc/netplan/netplan_wifi.yaml"



MACADDR_WIFI="`ip addr show wlan0 | awk '/ether/{print $2}' | sed 's/://g'`"
sed "s#<MACADDR>#${MACADDR_WIFI}#g" "${template_netplan_file}" | sudo tee "${netplan_file}"
sudo netplan generate

if [ $? -ne 0 ]; then 
	echo -e "netplan generate Error\nAbort..."
	exit
fi
sudo netplan apply

# NETWORKD config ETH ip fixe (no carrer)

template_eth_net_file="${ROBAIR_HOME}/configs/10-netplan-eth0.network.template"
eth_net_file="/etc/systemd/network/10-netplan-eth0.network"

MACADDR_ETH="`ip addr show eth0 | grep ether | awk -F ' ' '{print $2}'`"

sed "s#<ETH_MAC_ADDR>#${MACADDR_ETH}#g" "${template_eth_net_file}"  | sudo tee "${eth_net_file}"

sudo systemctl restart systemd-networkd

echo "network configuration DONE"
