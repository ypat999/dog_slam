#!/bin/bash

CONFIG_FILE="/etc/sysctl.conf"
PARAM="net.ipv4.ip_forward"
TEMP_FILE=$(mktemp)

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "$CONFIG_FILE not exist!"
    exit 1
fi

# If line exists but commented (# net.ipv4.ip_forward=1)
if grep -Eq "^\s*#\s*$PARAM\s*=\s*1" "$CONFIG_FILE"; then
    cp "$CONFIG_FILE" "$CONFIG_FILE.bak"
    sed -E "s/^\s*#\s*$PARAM\s*=\s*1/$PARAM=1/" "$CONFIG_FILE" > "$TEMP_FILE"
    # If already enabled
elif grep -Eq "^\s*$PARAM\s*=\s*1" "$CONFIG_FILE"; then
    echo "$PARAM already enabled."
# If disabled (net.ipv4.ip_forward=0)
elif grep -Eq "^\s*$PARAM\s*=\s*0" "$CONFIG_FILE"; then
    cp "$CONFIG_FILE" "$CONFIG_FILE.bak"
    sed -E "s/^\s*$PARAM\s*=\s*0/$PARAM=1/" "$CONFIG_FILE" > "$TEMP_FILE"
# If parameter not found at all
else
    echo "$PARAM=1" >> "$CONFIG_FILE"
    TEMP_FILE="$CONFIG_FILE"
fi

# Replace file if needed
if [[ -s "$TEMP_FILE" && "$TEMP_FILE" != "$CONFIG_FILE" ]]; then
    mv "$TEMP_FILE" "$CONFIG_FILE"
fi

# Apply changes
sysctl -p

# Cleanup
[[ -f "$TEMP_FILE" && "$TEMP_FILE" != "$CONFIG_FILE" ]] && rm -f "$TEMP_FILE"


## for LAN to WAN
update-alternatives --set iptables /usr/sbin/iptables-legacy
update-alternatives --set ip6tables /usr/sbin/ip6tables-legacy
#iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
#iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
#iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
#netfilter-persistent save

## for WAN to LAN
#iptables -t nat -A PREROUTING -p tcp -i wlan0 --dport 23 -j DNAT --to-destination 192.168.168.100:22

iptables -t nat -F
iptables -F FORWARD

# NAT to Internet via wlan0
iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE

# Forward traffic from eth0 → wlan0
iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT

# Forward traffic from p2p0 → wlan0
iptables -A FORWARD -i p2p0 -o wlan0 -j ACCEPT
iptables -A FORWARD -i wlan0 -o p2p0 -m state --state RELATED,ESTABLISHED -j ACCEPT

# Allow routing between eth0 and p2p0 (so PC ↔ MACHINE2 can talk)
iptables -A FORWARD -i eth0 -o p2p0 -j ACCEPT
iptables -A FORWARD -i p2p0 -o eth0 -j ACCEPT

