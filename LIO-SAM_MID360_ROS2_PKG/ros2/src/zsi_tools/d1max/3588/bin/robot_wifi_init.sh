#!/bin/bash
ENV_FILE=/userdata/config/wifi_config.bash
UDEV_RULE=/etc/udev/rules.d/90-robot-wifi.rules

ENV_FILE_DIR=$(dirname $ENV_FILE)
AP_DEVICE="p2p0"
STA_DEVICE="wlan0"

if [ ! -f $UDEV_RULE ]; then
cat > $UDEV_RULE << EOF
ACTION=="add", SUBSYSTEM=="net", KERNEL=="p2p0", RUN+="/opt/runtime/bin/robot_wifi_init.sh"
EOF
chmod 644 $UDEV_RULE
fi

function init_config() {
    echo "AP_SSID=XXXX_AX" > $ENV_FILE
    echo "AP_PASSWD=12345678" >> $ENV_FILE
    echo "AP_IP=192.168.234.1/24" >> $ENV_FILE
    echo "COUNTRY_CODE=CN" >> $ENV_FILE
    echo "AP_ENABLE=1" >> $ENV_FILE
    echo "AP_5G_ENABLE=1" >> $ENV_FILE
    echo "AP_CHAN_50G=0" >> $ENV_FILE
    echo "AP_CHANLIST_50G=\"36-48 149-165\"" >> $ENV_FILE
    echo "AP_CHAN_24G=0" >> $ENV_FILE
    echo "AP_CHANLIST_24G=\"1-13\"" >> $ENV_FILE
    echo "AP_AUTO_PREFIX=1" >> $ENV_FILE
    echo "STA_ENABLE=0" >> $ENV_FILE
}

if [ ! -d $ENV_FILE_NAME  ] || [ ! -f $ENV_FILE ] ; then
    mkdir -p $ENV_FILE_DIR
    init_config
fi

. $ENV_FILE
if [ ! -n "$STA_ENABLE" ]; then
    init_config
    . $ENV_FILE
fi


AP_NETWORK_CONFIG=/etc/systemd/network/20-$AP_DEVICE.network
CONFIG_IP=$(cat $AP_NETWORK_CONFIG 2>/dev/null | awk -F'=' '/^Address=/ {print $2}')
if [ ! -f $AP_NETWORK_CONFIG ] || [ "$CONFIG_IP" != "$AP_IP" ]; then
echo "warning!! $CONFIG_IP" != "$AP_IP"
cat > $AP_NETWORK_CONFIG << EOF
[Match]
Name=$AP_DEVICE

[Network]
Address=$AP_IP
DHCPServer=yes

[DHCPServer]
PoolOffset=200
PoolSize=50
EmitRouter=yes
DNS=$(echo $AP_IP | cut -d'/' -f1)
EOF
chmod 664 /etc/systemd/network/20-$AP_DEVICE.network
networkctl reload
networkctl reconfigure $AP_DEVICE
fi

STA_NETWORK_CONFIG=/etc/systemd/network/25-$STA_DEVICE.network
if [ ! -f $STA_NETWORK_CONFIG ]; then
cat > $STA_NETWORK_CONFIG << EOF
[Match]
Name=$STA_DEVICE

[Network]
DHCP=yes
EOF
chmod 664 /etc/systemd/network/25-$STA_DEVICE.network
fi

UNMANAGE_CONF=/etc/NetworkManager/conf.d/unmanaged.conf   
if ! grep -q "interface-name:wlan0" $UNMANAGE_CONF || ! grep -q "interface-name:p2p0" $UNMANAGE_CONF; then
cat > /etc/NetworkManager/conf.d/unmanaged.conf << EOF
[keyfile]
unmanaged-devices=interface-name:ap0;interface-name:wlan0;interface-name:p2p0
EOF
    nmcli device set wlan0 managed no
    nmcli device set p2p0 managed no
fi

WDG_SERVICE=/etc/systemd/system/wifi-wdg.service
HOSTAPD_SERVICE=/etc/systemd/system/hostapd.service

if [ ! -f $WDG_SERVICE ]; then

cat > $WDG_SERVICE << EOF
[Unit]
Description=Keep WIFI AP alive
After=hostapd.service

[Service]
Type=simple
ExecStart=/opt/runtime/bin/robot_wifi_wdg.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

cat > $HOSTAPD_SERVICE << EOF
[Unit]
Description=Hostapd IEEE 802.11 AP, IEEE 802.1X/WPA/WPA2/EAP/RADIUS Authenticator
After=network.target
StartLimitIntervalSec=60s
StartLimitBurst=3

[Service]
ExecStart=/usr/sbin/hostapd /tmp/hostapd.conf
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
EOF
/usr/bin/systemctl daemon-reload

fi

if [ $AP_5G_ENABLE -ne 0 ]; then
    HW_MODE=a
    CHAN=$AP_CHAN_50G
    CHAHLIST=$AP_CHANLIST_50G
    NEED_AC=1
    HW_CHWD=1
    if [ $AP_AUTO_PREFIX -ne 0 ]; then
    	AP_SSID=ZG50G_$AP_SSID
    fi
else
    HW_MODE=g
    CHAN=$AP_CHAN_24G
    CHAHLIST=$AP_CHANLIST_24G
    NEED_AC=0
    HW_CHWD=0
    if [ $AP_AUTO_PREFIX -ne 0 ]; then
    	AP_SSID=ZG24G_$AP_SSID
    fi
fi

cat > /tmp/hostapd.conf << EOF
interface=$AP_DEVICE
ctrl_interface=/var/run/hostapd
ssid=$AP_SSID
driver=nl80211
beacon_int=100

hw_mode=$HW_MODE
channel=$CHAN
chanlist=$CHAHLIST

acs_num_scans=5
acs_exclude_dfs=1

ieee80211n=1
require_ht=1
ht_capab=[HT40+][SHORT-GI-20][SHORT-GI-40][RX-STBC][RX-STBC1][TX-STBC][TX-STBC-2BY1][DSSS_CCK-40]

ieee80211ac=$NEED_AC
require_vht=1
vht_oper_chwidth=1
#vht_oper_centr_freq_seg0_idx=155
vht_capab=[MAX-MPDU-7991][SHORT-GI-80][TX-STBC-2BY1][SU-BEAMFORMEE][MU-BEAMFORMEE][HTC-VHT]

ieee80211ax=1
he_oper_chwidth=$HW_CHWD
#he_oper_centr_freq_seg0_idx=6
he_su_beamformer=1
he_su_beamformee=1
he_mu_beamformer=1
he_bss_color=11

auth_algs=3
ieee80211w=2

wpa_passphrase=$AP_PASSWD
wpa=2
wpa_key_mgmt=WPA-PSK SAE
wpa_pairwise=CCMP
rsn_pairwise=CCMP
max_num_sta=8

wmm_enabled=1
EOF

if ! systemctl is-enabled systemd-networkd >/dev/null 2>&1; then
    /usr/bin/systemctl enable systemd-networkd
    /usr/bin/systemctl restart systemd-networkd
fi

if ! systemctl is-enabled systemd-resolved >/dev/null 2>&1; then
    /usr/bin/systemctl enable systemd-resolved	
    /usr/bin/systemctl restart systemd-resolved
fi
if [ ! -L /etc/resolv.conf ]; then
    rm -f /etc/resolv.conf
    ln -s /run/systemd/resolve/stub-resolv.conf /etc/resolv.conf
fi
/usr/sbin/iw reg set $COUNTRY_CODE
/usr/bin/systemctl disable hostapd.service > /dev/null
/usr/bin/systemctl disable wifi-wdg.service > /dev/null
/usr/bin/systemctl disable wpa_supplicant.service > /dev/null
/usr/bin/systemctl stop hostapd.service > /dev/null
/usr/bin/systemctl stop wifi-wdg.service > /dev/null
/usr/bin/systemctl stop wpa_supplicant.service > /dev/null
if [ $AP_ENABLE -ne 0 ]; then 
    /usr/bin/systemctl restart hostapd.service
fi
if [ $STA_ENABLE -ne 0 ]; then
    /usr/bin/systemctl start wpa_supplicant.service
fi
/usr/bin/systemctl restart wifi-wdg.service
exit 0
