#!/bin/bash
AP_DEVICE="p2p0"
SSID_50G_PREFIX=""
WIFI_CONFIG_FILE=/userdata/config/wifi_config.bash

PROGRAM_NAME=$(basename $0)
usage() {
    echo "usage: $PROGRAM_NAME status"
    echo "usage: $PROGRAM_NAME stop"
    echo "usage: $PROGRAM_NAME start"
    echo "usage: $PROGRAM_NAME enable"
    echo "usage: $PROGRAM_NAME disable"
    echo "usage: $PROGRAM_NAME create <ssid> <password>"
}

if [ $# -eq 1 ]; then
    case $1 in
	    "start")
    		/usr/bin/systemctl start hostapd.service
		;;
	    "stop")
    		/usr/bin/systemctl stop hostapd.service
		;;
        "enable")
		sed -i -e "s/^AP_ENABLE=.*/AP_ENABLE=1/" $WIFI_CONFIG_FILE
		;;
        "disable")
		sed -i -e "s/^AP_ENABLE=.*/AP_ENABLE=0/" $WIFI_CONFIG_FILE
		;;
        "status")
    		echo "Driver info:"
    		cat /proc/net/rtl8852be/$AP_DEVICE/ap_info
		;;
	*) echo "cmd $1 invalid !!"
		exit 5;
		usage
		;;
    esac
    exit 0;
elif [ $# -eq 3 ]; then
    case $1 in
        "create")
        sed -i -e "s/^AP_SSID=.*/AP_SSID=${SSID_50G_PREFIX}$2/" \
       -e "s/^AP_PASSWD=.*/AP_PASSWD=$3/" $WIFI_CONFIG_FILE
		;;
	*) echo "cmd $1 invalid !!"
		exit 8;
		usage
		;;
    esac
    exit 0;
else
    echo "invalid cmd $@"
    usage
    exit 9;
fi


