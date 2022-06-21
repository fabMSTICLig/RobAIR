#!/bin/bash
IP_ETH=192.168.0.175
MASK_ETH=24
# network manager doit etre installe avant

if ! which NetworkManager > /dev/null; then
  if ! sudo apt install -y network-manager ; then
		echo -e "network manager must be install\nAbort..."
		exit 1
	fi
fi

# desactive la configuration netplan actuelle

shopt -s nullglob
for file in /etc/netplan/*.yaml ; do
	sudo mv "${file}" "${file}.bak" ;
done

# NETPLAN config WIFI HotSpot
template_netplan_file="${ROBAIR_HOME}/configs/netplan_wifi.yaml.template"
netplan_file="/etc/netplan/netplan_wifi.yaml"



MACADDR_WIFI="$(ip addr show wlan0 | awk '/ether/{print $2}' | sed 's/://g')"
sed "s#<MACADDR>#${MACADDR_WIFI}#g" "${template_netplan_file}" | sudo tee "${netplan_file}"

if ! sudo netplan generate ; then 
	echo -e "netplan generate Error\nAbort..."
	exit
fi
sudo netplan apply

# NETWORKD config ETH ip fixe (no carrer)

template_eth_net_file="${ROBAIR_HOME}/configs/10-netplan-eth0.network.template"
eth_net_file="/etc/systemd/network/10-netplan-eth0.network"

#MACADDR_ETH="$(ip addr show eth0 | grep ether | awk -F ' ' '{print $2}')"

sed "s#<IP_ETH>#${IP_ETH}#;s#<MASK_ETH>#${MASK_ETH}#" "${template_eth_net_file}" | sudo tee "${eth_net_file}"

sudo systemctl restart systemd-networkd

echo "network configuration DONE"
