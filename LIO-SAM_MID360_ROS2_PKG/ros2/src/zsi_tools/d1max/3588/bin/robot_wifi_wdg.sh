#!/bin/bash
: "${DEBUG:=0}"
IRQ_SRC=/sys/module/8852be/drivers/pci\:rtl8852be/0004\:41\:00.0/irq
CHECK_INTERVAL=10
CHECK_COUNT=2
MIN_IRQS=1
INTERFACE="p2p0"

ERROR_COUNT=0
IRQ_NO=-1
IRQ_COUNT=0
PREV_IRQ_COUNT=-1
LOG_STR=""
AP_SERVICE=hostapd.service
STA_SERVICE=wpa_supplicant.service

. /userdata/config/wifi_config.bash

errlog() {
 [ $DEBUG -ne 0 ] && echo $*
 logger --stderr -t "APWDG" $*
}

abs_diff() {
    echo $(($1 >= $2 ? $1 - $2 : $2 - $1))
}

state_reset() {
    ERROR_COUNT=0
    IRQ_NO=-1
    IRQ_COUNT=0
    PREV_IRQ_COUNT=-1
    LOG_STR=""
}

wifi_operation() {
    if [ $ERROR_COUNT -lt $CHECK_COUNT ]; then
        return 0
    fi
    echo 1000 > /sys/devices/platform/fe190000.pcie/reset_ms
    echo 1 > /sys/devices/platform/fe190000.pcie/reset
    sleep 5
    #timeout 5 modprobe -r -f 8852be > /dev/null
    if [ -f /sys/devices/platform/fe190000.pcie/pci0004:40/0004:40:00.0/remove ]; then
        timeout 5 echo 1 > /sys/devices/platform/fe190000.pcie/pci0004:40/0004:40:00.0/remove
        sleep 1
    fi
    timeout 10 echo 1 > /sys/bus/pci/rescan
    sleep 1
    #timeout 10 /bin/systemctl reset-failed $AP_SERVICE > /dev/null
    #timeout 10 /bin/systemctl reset-failed $STA_SERVICE > /dev/null
    #timeout 10 /opt/runtime/bin/robot_wifi_init.sh
    sleep 1
    timeout 10 networkctl reconfigure p2p0 wlan0
    state_reset
    sleep 5
    return 0
}

check_driver_alive() {
    if [ ! -f ${IRQ_SRC} ]; then
	LOG_STR="${IRQ_SRC}"" not exist!!"
        errlog "$LOG_STR"
        return 1
    fi
    IRQ_NO=$(cat ${IRQ_SRC})
    if [ $? -ne 0 ] || [ ${IRQ_NO} -eq -1 ]; then
        LOG_STR="get irq from ""${IRQ_SRC}"" failed!!"
        errlog "$LOG_STR"
	return 1
    fi
    LINE_STR=$(head -n1 /proc/irq/$IRQ_NO/spurious)
    if [ $? -ne 0 ]; then
        LOG_STR="get info from ""/proc/irq/$IRQ_NO/spurious"" failed!!"
        errlog "$LOG_STR"
	return 2 
    fi 
    IRQ_COUNT=$(echo $LINE_STR | awk '{print $2}')
    DIFF=$(abs_diff $IRQ_COUNT $PREV_IRQ_COUNT)
    PREV_IRQ_COUNT=$IRQ_COUNT
    [ $DEBUG -ne 0 ] && echo $DIFF
    if [ $DIFF -lt $MIN_IRQS ]; then
	    LOG_STR="irq count $DIFF in "$CHECK_INTERVAL"s is not normal (normal: $MIN_IRQS)"
        errlog "$LOG_STR"
	return 3 
    fi
    return 0
}

check_interface_alive() {
    if [ ! -d /sys/class/net/$INTERFACE ]; then
        LOG_STR="$INTERFACE not exist!!"
	errlog "$LOG_STR"
        return 1
    fi

    STATE=$(timeout 1 cat /sys/class/net/p2p0/operstate)
    if [ $? -ne 0 ] || [ ! "$STATE" = "up" ]; then
        LOG_STR="$INTERFACE not UP!!"
	errlog "$LOG_STR"
	return 2
    fi

    IPV4_ADDR=$(timeout 5 ip addr show p2p0 2>/dev/null | awk '/inet / {print $2; exit}')
    if [ $? -ne 0 ]; then
        LOG_STR="failed to get IP address from $INTERFACE"
	errlog "$LOG_STR"
	return 3
    elif [ ! -n "$IPV4_ADDR" ]; then
        LOG_STR="no IP address assign to $INTERFACE"
	errlog "$LOG_STR"
	timeout 5 networkctl reconfigure p2p0
    fi
    return 0
}

while true
do
    sleep $CHECK_INTERVAL
    #check first
    check_driver_alive
    if [ $? -ne 0 ]; then
        ERROR_COUNT=$(expr $ERROR_COUNT + 1)
        errlog "ERR""$ERROR_COUNT"
	wifi_operation
	continue
    fi
    #check second
    check_interface_alive
    if [ $? -ne 0 ]; then
        ERROR_COUNT=$(expr $ERROR_COUNT + 1)
        errlog "ERR""$ERROR_COUNT"
	wifi_operation
	continue
    fi
    #check 3th

    if [ $AP_ENABLE -ne 0 ] &&  ! systemctl is-active $AP_SERVICE --quiet; then
        LOG_STR="$AP_SERVICE is dead!!"
	errlog $LOG_STR
    	timeout 10 /bin/systemctl reset-failed $AP_SERVICE > /dev/null
    	timeout 10 /bin/systemctl restart $AP_SERVICE > /dev/null
	#ERROR_COUNT=$(expr $ERROR_COUNT + 1)
        errlog "ERR""$ERROR_COUNT"
	#wifi_operation
	continue
    fi
    if [ $STA_ENABLE -ne 0 ] &&  ! systemctl is-active $STA_SERVICE --quiet; then
        LOG_STR="$STA_SERVICE is dead!!"
	errlog $LOG_STR
    	timeout 10 /bin/systemctl reset-failed $STA_SERVICE > /dev/null
    	timeout 10 /bin/systemctl restart $STA_SERVICE > /dev/null
	#ERROR_COUNT=$(expr $ERROR_COUNT + 1)
        errlog "ERR""$ERROR_COUNT"
	#wifi_operation
	continue
    fi
    ERROR_COUNT=0
done
