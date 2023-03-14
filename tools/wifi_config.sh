#!/bin/bash

usage()
{
cat <<EOF
Usage: $0 [options] wireless-device id-number
Options:
    -P POWER        Set txpower (default=10, set to 20 for flight ops)
EOF
}

# option defaults
INET_CONFIG=0
ROUTER_USE=0
ROUTER_DEV=""
TXPOWER=10

# Network defaults
TEAM_SUBNET=11
TEAM_CHAN=1
TEAM_AP="00:99:88:77:66:55"
TEAM_SSID="casa"

#parse options
while getopts ":2IR:P:T:h" opt; do
    case $opt in
        P)
            if [ -z $OPTARG ]; then
                usage
                exit 1
            fi
            TXPOWER=$OPTARG
            ;;
        h)
            usage
            exit 0
            ;;
    esac
done
shift $((OPTIND-1))

if [ -z $1 ] || [ -z $2 ]; then
  usage
  exit 1
fi

echo "setting up for $TEAM_SSID ..."

if [ $ROUTER_USE != 0 ]; then
  echo "Setting up router ..."
  sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
  sudo iptables -t nat -F  # Flush old rules first
  sudo iptables -t nat -A POSTROUTING -o $ROUTER_DEV -j MASQUERADE
fi

sudo ifconfig $1 down
sudo iwconfig $1 mode ad-hoc essid $TEAM_SSID channel $TEAM_CHAN \
  ap $TEAM_AP txpower $TXPOWER
sudo ifconfig $1 inet 192.168.$TEAM_SUBNET.$2/24 up

if [ $INET_CONFIG != 0 ]; then
  sudo route add default gw 192.168.${TEAM_SUBNET}.1
  echo -e "nameserver 172.20.20.11\nnameserver 8.8.8.8" | sudo tee /etc/resolv.conf
fi
