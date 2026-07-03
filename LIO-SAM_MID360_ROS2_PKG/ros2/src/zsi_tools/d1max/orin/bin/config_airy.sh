#!/bin/bash
BACK_AIRY_PATH=/sys/bus/usb/devices/1-3/1-3.3/1-3.3\:2.0/net/
FRONT_AIRY_PATH=/sys/bus/usb/devices/1-3/1-3.2/1-3.2\:2.0/net/
#set -x

function clean_nm_connections() {
    find /etc/NetworkManager/system-connections/ \
-maxdepth 1 \
-name "*.nmconnection" \
-exec grep -lZ "interface-name=$1" {} \; \
| xargs -r -0 -n1 basename -s .nmconnection \
| xargs -r -I {} nmcli connection delete "{}" > /dev/null 2>&1
ifconfig $1 down
}

echo "开始配置..."
net_front=$(ls $FRONT_AIRY_PATH | head -n 1)
if [ $? -eq 0 ] && [ -n "$net_front" ]; then
    echo "找到前雷达接口 $net_front"
else
    echo "未找到前雷达接口！"
fi
echo "寻找后雷达接口..."
net_rear=$(ls $BACK_AIRY_PATH | head -n 1)
if [ $? -eq 0 ] && [ -n "$net_rear" ]; then
    echo "找到后雷达接口 $net_rear"
else
    echo "未找到后雷达接口！"
fi

declare -A front_airy=(
    ["SrcIp"]="192.168.1.200"
    ["SrcMask"]="255.255.255.0" 
    ["SrcGateWay"]="192.168.1.1"
    ["DstIp"]="192.168.1.102"
    ["MPort"]="6699"
    ["DPort"]="7788"
    ["ReMode"]="0"
    ["SynSrc"]="0"
    ["PDomNum"]="0"
    ["RTPdelay"]="0"
    ["NoLeap"]="0"
    ["SyTOut"]="5"
    ["UToL"]="1"
    ["LToU"]="20"
    ["OpM"]="1"
    ["PhaLockSet"]="0"
    ["GPSBRate"]="10"
    ["ImuCtrl"]="1"
    ["IPort"]="6688"
    ["ImuOutR"]="3"
    ["AccRg"]="1"
    ["GyroRg"]="1"
    ["Lpf"]="0"
    ["ResDefault"]="0"
    ["RefEn"]="0"
    ["TFilLvl"]="6"
    ["BlkRnDis"]="0"
    ["Rn"]="0"
    ["Blk"]="0"
    ["FSAgl"]="180"
    ["DZ10cm"]="1"
    ["C81858993"]="0"
    ["GapFill"]="0"
)

declare -A rear_airy=(
    ["SrcIp"]="192.168.2.200"
    ["SrcMask"]="255.255.255.0" 
    ["SrcGateWay"]="192.168.2.1"
    ["DstIp"]="192.168.2.102"
    ["MPort"]="6699"
    ["DPort"]="7788"
    ["ReMode"]="0"
    ["SynSrc"]="0"
    ["PDomNum"]="0"
    ["RTPdelay"]="0"
    ["NoLeap"]="0"
    ["SyTOut"]="5"
    ["UToL"]="1"
    ["LToU"]="20"
    ["OpM"]="1"
    ["PhaLockSet"]="0"
    ["GPSBRate"]="10"
    ["ImuCtrl"]="1"
    ["IPort"]="6688"
    ["ImuOutR"]="3"
    ["AccRg"]="1"
    ["GyroRg"]="1"
    ["Lpf"]="0"
    ["ResDefault"]="0"
    ["RefEn"]="0"
    ["TFilLvl"]="6"
    ["BlkRnDis"]="0"
    ["Rn"]="0"
    ["Blk"]="0"
    ["FSAgl"]="180"
    ["DZ10cm"]="1"
    ["C81858993"]="0"
    ["GapFill"]="0"
)

check_front_variables() {
    local mismatch_count=0
    # 检查每个变量
    for var_name in "${!front_airy[@]}"; do
        expected="${front_airy[$var_name]}"
        # 使用间接引用来获取变量的实际值
        actual="${!var_name}"
        if [ "$actual" != "$expected" ]; then
            echo "变量 $var_name 不匹配: 期望='$expected', 实际='$actual'" >&2
            ((mismatch_count++))
        else
            echo "变量 $var_name 匹配: 期望='$expected', 实际='$actual'" >&2
        fi
    done
    echo "前雷达配置检查完成，共有 $mismatch_count 处不匹配" >&2
    return $mismatch_count
}

check_rear_variables() {
    local mismatch_count=0
    # 检查每个变量
    for var_name in "${!rear_airy[@]}"; do
        expected="${rear_airy[$var_name]}"
        # 使用间接引用来获取变量的实际值
        actual="${!var_name}"
        if [ "$actual" != "$expected" ]; then
            echo "变量 $var_name 不匹配: 期望='$expected', 实际='$actual'" >&2
            ((mismatch_count++))
        else
            echo "变量 $var_name 匹配: 期望='$expected', 实际='$actual'" >&2
        fi
    done
    echo "后雷达配置检查完成，共有 $mismatch_count 处不匹配" >&2
    return $mismatch_count
}

DESC_Return_Mode=("Strongest" "First" "Last" "Dual")
DESC_SynSrc=("GPS" "PTP-E2E" "PTP-P2P" "PTP-GPTP" "PTP-E2E-L2")
DESC_ONOFF=("OFF" "ON")
DESC_OpM=("Standby" "High-Performance")
DESC_GPSBRate=("" "" "" "9600" "14400" "19200" "38400" "43200" "57600" "76800" "115200")
DESC_ImuOutR=("25Hz 50Hz 100Hz 200Hz")
DESC_AccRg=("[-2g,2g]" "[-4g,4g]" "[-8g,8g]" "[-16g,16g]")
DESC_GyroRg=("[-250,250]dps" "[-500,500]dps" "[-1000,1000]dps" "[-2000,2000]dps")
DESC_RefEn=("OFF" "ON1" "ON2" "ON3")
DESC_BlkRnDis=("30cm" "20cm" "10cm")
DESC_Rn=("high" "medium" "low")
DESC_Blk=("high" "medium" "low")
DSEC_DZ10cm=("ON" "OFF")



function get_airy_config() {
    SrcIp=""
    SrcMask=""
    SrcGateWay=""
    DstIp=""
    MPort=""
    DPort=""
    ReMode=""
    SynSrc=""
    PDomNum=""
    RTPdelay=""
    NoLeap=""
    SyTOut=""
    UToL=""
    LToU=""
    OpM=""
    PhaLockSet=""
    GPSBRate=""
    ImuCtrl=""
    IPort=""
    ImuOutR=""
    AccRg=""
    GyroRg=""
    Lpf=""
    ResDefault=""
    RefEn=""
    TFilLvl=""
    BlkRnDis=""
    Rn=""
    Blk=""
    FSAgl=""
    DZ10cm=""
    C81858993=""
    GapFill=""
    PoorMask=""
    RES=$(curl -s http://$1/setting_data.json)
    if [ $? -ne 0 ]; then
        #echo "Get Airy Config Failed"
        return 1
    fi
    eval $(echo $RES| sed 's/{//;s/}//;s/"//g; s/,/\n/g; s/:/=/g')
    return $?
}

function print_airy_config() {
    echo "Device IP Address:                      " $SrcIp
    echo "Device IP Mask:                         " $SrcMask
    echo "Device IP Gateway:                      " $SrcGateWay
    echo "Destination IP Address:                 " $DstIp
    echo "MSOP Port Number(1025~65535):           " $MPort
    echo "DIFOP Port Number(1025~65535):          " $DPort
    echo "Return Mode:                            " ${DESC_Return_Mode[$ReMode]} #$ReMode 
    echo "Time Synchronization Source:            " ${DESC_SynSrc[$SynSrc]} #$SynSrc 
    echo "PTP Domain Number(0~127):               " $PDomNum
    echo "Respond To PeerDelayRequest:            " ${DESC_ONOFF[$RTPdelay]} #$RTPdelay 
    echo "No Leap Second:                         " ${DESC_ONOFF[$NoLeap]} #$NoLeap 
    echo "Timeout(1~255s):                        " $SyTOut
    echo "Unlock To Lock Threshold(1~255ms):      " $UToL
    echo "Lock To Unlock Threshold(1~255ms):      " $LToU
    echo "Operation Mode:                         " ${DESC_OpM[$OpM]} #$OpM 
    echo "Phase Lock Setting(0~360):              " $PhaLockSet "DEG"
    echo "GPS Baud Rate:                          " ${DESC_GPSBRate[$GPSBRate]} $GPSBRate
    echo "Imu Ctrl:                               " ${DESC_ONOFF[$ImuCtrl]} #$ImuCtrl 
    echo "Imu Port Number(1025~65535):            " $IPort
    echo "Imu Output Rate:                        " ${DESC_ImuOutR[$ImuOutR]} #$ImuOutR 
    echo "Accel Range:                            " ${DESC_AccRg[$AccRg]} #$AccRg 
    echo "Gyro Range:                             " ${DESC_GyroRg[$GyroRg]} #$GyroRg 
    echo "Imu LPF Config(0~255):                  " $Lpf
    echo "Restore Default:                        " ${DESC_ONOFF[$ResDefault]} #$ResDefault 
    echo "Reflectivity Enhancement:               " ${DESC_RefEn[$RefEn]} #$RefEn  
    echo "Trail Filter Level::                    " $TFilLvl 
    echo "Rain/Blockage Detection Distance:       " ${DESC_BlkRnDis[$BlkRnDis]} #$BlkRnDis 
    echo "Rn:                                     " ${DESC_Rn[$Rn]} #$Rn 
    echo "Blk:                                    " ${DESC_Blk[$Blk]} #$Blk
    echo "FSAgl:                                  " $FSAgl
    echo "DZ10cm:                                 " ${DSEC_DZ10cm[$DZ10cm]} #$DZ10cm 
    echo "C81858993:                              " ${DESC_ONOFF[$C81858993]} #$C81858993
    echo "GapFill:                                " ${DESC_ONOFF[$GapFill]} #$GapFill
}


# 检查当前前雷达网络和设备参数是否已经是目标配置
is_front_config_ok() {
    if [ -z "$net_front" ]; then
        return 1
    fi

    local current_ip
    current_ip=$(ip -4 addr show dev "$net_front" | awk '/inet /{print $2}' | head -n1 | cut -d/ -f1)
    if [ "$current_ip" != "${front_airy["DstIp"]}" ]; then
        return 1
    fi

    get_airy_config ${front_airy["SrcIp"]}
    if [ $? -ne 0 ]; then
        return 1
    fi

    check_front_variables
    return $?
}

# 检查当前后雷达网络和设备参数是否已经是目标配置
is_rear_config_ok() {
    if [ -z "$net_rear" ]; then
        return 1
    fi

    local current_ip
    current_ip=$(ip -4 addr show dev "$net_rear" | awk '/inet /{print $2}' | head -n1 | cut -d/ -f1)
    if [ "$current_ip" != "${rear_airy["DstIp"]}" ]; then
        return 1
    fi

    get_airy_config ${rear_airy["SrcIp"]}
    if [ $? -ne 0 ]; then
        return 1
    fi

    check_rear_variables
    return $?
}

# 检查 NetworkManager 中 front 连接是否已经是最终期望配置
is_front_nm_ok() {
    # 连接不存在则认为不满足，需要后续创建/修改
    if ! nmcli -t -f NAME connection show 2>/dev/null | grep -qx "front"; then
        return 1
    fi

    local gw never ip
    gw=$(nmcli -g ipv4.gateway connection show front 2>/dev/null)
    never=$(nmcli -g ipv4.never-default connection show front 2>/dev/null)
    ip=$(nmcli -g ipv4.addresses connection show front 2>/dev/null | head -n1 | cut -d/ -f1)

    if [ -z "$gw" ] && [ "$never" = "yes" ] && [ "$ip" = "${front_airy["DstIp"]}" ]; then
        return 0
    fi

    return 1
}

# 检查 NetworkManager 中 rear 连接是否已经是最终期望配置
is_rear_nm_ok() {
    # 连接不存在则认为不满足，需要后续创建/修改
    if ! nmcli -t -f NAME connection show 2>/dev/null | grep -qx "rear"; then
        return 1
    fi

    local gw never ip
    gw=$(nmcli -g ipv4.gateway connection show rear 2>/dev/null)
    never=$(nmcli -g ipv4.never-default connection show rear 2>/dev/null)
    ip=$(nmcli -g ipv4.addresses connection show rear 2>/dev/null | head -n1 | cut -d/ -f1)

    if [ -z "$gw" ] && [ "$never" = "yes" ] && [ "$ip" = "${rear_airy["DstIp"]}" ]; then
        return 0
    fi

    return 1
}


function set_front_airy_config() {
curl -s -L \
  --connect-timeout 5 \
  --max-time 10 \
  -H "Connection: close" \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -H "Referer: http://$1/Parameter_Setting.html" \
  --data "tabs-two=on&\
SrcIp=${front_airy["SrcIp"]}&\
SrcMask=${front_airy["SrcMask"]}&\
SrcGateWay=${front_airy["SrcGateWay"]}&\
DstIp=${front_airy["DstIp"]}&\
MPort=${front_airy["MPort"]}&\
DPort=${front_airy["DPort"]}&\
ReMode=${front_airy["ReMode"]}&\
SynSrc=${front_airy["SynSrc"]}&\
PDomNum=${front_airy["PDomNum"]}&\
RTPdelay=${front_airy["RTPdelay"]}&\
NoLeap=${front_airy["NoLeap"]}&\
SyTOut=${front_airy["SyTOut"]}&\
UToL=${front_airy["UToL"]}&\
LToU=${front_airy["LToU"]}&\
OpM=${front_airy["OpM"]}&\
PhaLockSet=${front_airy["PhaLockSet"]}&\
GPSBRate=${front_airy["GPSBRate"]}&\
ImuCtrl=${front_airy["ImuCtrl"]}&\
IPort=${front_airy["IPort"]}&\
ImuOutR=${front_airy["ImuOutR"]}&\
AccRg=${front_airy["AccRg"]}&\
GyroRg=${front_airy["GyroRg"]}&\
Lpf=${front_airy["Lpf"]}&\
ResDefault=${front_airy["ResDefault"]}&\
save_param=Save&\
RefEn=${front_airy["RefEn"]}&\
TFilLvl=${front_airy["TFilLvl"]}&\
BlkRnDis=${front_airy["BlkRnDis"]}&\
Rn=${front_airy["Rn"]}&\
Blk=${front_airy["Blk"]}&\
FSAgl=${front_airy["FSAgl"]}&\
DZ10cm=${front_airy["DZ10cm"]}&\
C81858993=${front_airy["C81858993"]}&\
GapFill=${front_airy["GapFill"]}&\
mode2=Mode2" \
http://$1/Parameter_Setting.html \
-o /dev/null || true
sleep 5
}

function set_front_airy_config_performance() {
curl -s -L \
  --connect-timeout 5 \
  --max-time 10 \
  -H "Connection: close" \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -H "Referer: http://$1/Parameter_Setting.html" \
  --data "\
SrcIp=${front_airy["SrcIp"]}&\
SrcMask=${front_airy["SrcMask"]}&\
SrcGateWay=${front_airy["SrcGateWay"]}&\
DstIp=${front_airy["DstIp"]}&\
MPort=${front_airy["MPort"]}&\
DPort=${front_airy["DPort"]}&\
ReMode=${front_airy["ReMode"]}&\
SynSrc=${front_airy["SynSrc"]}&\
PDomNum=${front_airy["PDomNum"]}&\
RTPdelay=${front_airy["RTPdelay"]}&\
NoLeap=${front_airy["NoLeap"]}&\
SyTOut=${front_airy["SyTOut"]}&\
UToL=${front_airy["UToL"]}&\
LToU=${front_airy["LToU"]}&\
OpM=${front_airy["OpM"]}&\
PhaLockSet=${front_airy["PhaLockSet"]}&\
GPSBRate=${front_airy["GPSBRate"]}&\
ImuCtrl=${front_airy["ImuCtrl"]}&\
IPort=${front_airy["IPort"]}&\
ImuOutR=${front_airy["ImuOutR"]}&\
AccRg=${front_airy["AccRg"]}&\
GyroRg=${front_airy["GyroRg"]}&\
Lpf=${front_airy["Lpf"]}&\
ResDefault=${front_airy["ResDefault"]}&\
tabs-two=on&\
RefEn=${front_airy["RefEn"]}&\
TFilLvl=${front_airy["TFilLvl"]}&\
BlkRnDis=${front_airy["BlkRnDis"]}&\
Rn=${front_airy["Rn"]}&\
Blk=${front_airy["Blk"]}&\
FSAgl=${front_airy["FSAgl"]}&\
DZ10cm=${front_airy["DZ10cm"]}&\
C81858993=${front_airy["C81858993"]}&\
GapFill=${front_airy["GapFill"]}&\
save_param_performance=Save&\
mode2=Mode2" \
http://$1/Parameter_Setting.html \
-o /dev/null || true
sleep 5
}

function set_rear_airy_config() {
curl -s -L \
  --connect-timeout 5 \
  --max-time 10 \
  -H "Connection: close" \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -H "Referer: http://$1/Parameter_Setting.html" \
  --data "tabs-two=on&\
SrcIp=${rear_airy["SrcIp"]}&\
SrcMask=${rear_airy["SrcMask"]}&\
SrcGateWay=${rear_airy["SrcGateWay"]}&\
DstIp=${rear_airy["DstIp"]}&\
MPort=${rear_airy["MPort"]}&\
DPort=${rear_airy["DPort"]}&\
ReMode=${rear_airy["ReMode"]}&\
SynSrc=${rear_airy["SynSrc"]}&\
PDomNum=${rear_airy["PDomNum"]}&\
RTPdelay=${rear_airy["RTPdelay"]}&\
NoLeap=${rear_airy["NoLeap"]}&\
SyTOut=${rear_airy["SyTOut"]}&\
UToL=${rear_airy["UToL"]}&\
LToU=${rear_airy["LToU"]}&\
OpM=${rear_airy["OpM"]}&\
PhaLockSet=${rear_airy["PhaLockSet"]}&\
GPSBRate=${rear_airy["GPSBRate"]}&\
ImuCtrl=${rear_airy["ImuCtrl"]}&\
IPort=${rear_airy["IPort"]}&\
ImuOutR=${rear_airy["ImuOutR"]}&\
AccRg=${rear_airy["AccRg"]}&\
GyroRg=${rear_airy["GyroRg"]}&\
Lpf=${rear_airy["Lpf"]}&\
ResDefault=${rear_airy["ResDefault"]}&\
save_param=Save&\
RefEn=${rear_airy["RefEn"]}&\
TFilLvl=${rear_airy["TFilLvl"]}&\
BlkRnDis=${rear_airy["BlkRnDis"]}&\
Rn=${rear_airy["Rn"]}&\
Blk=${rear_airy["Blk"]}&\
FSAgl=${rear_airy["FSAgl"]}&\
DZ10cm=${rear_airy["DZ10cm"]}&\
C81858993=${rear_airy["C81858993"]}&\
GapFill=${rear_airy["GapFill"]}&\
mode2=Mode2" \
http://$1/Parameter_Setting.html \
-o /dev/null || true
sleep 5
}

function set_rear_airy_config_performance() {
curl -s -L \
  --connect-timeout 5 \
  --max-time 10 \
  -H "Connection: close" \
  -H "Content-Type: application/x-www-form-urlencoded" \
  -H "Referer: http://$1/Parameter_Setting.html" \
  --data "\
SrcIp=${rear_airy["SrcIp"]}&\
SrcMask=${rear_airy["SrcMask"]}&\
SrcGateWay=${rear_airy["SrcGateWay"]}&\
DstIp=${rear_airy["DstIp"]}&\
MPort=${rear_airy["MPort"]}&\
DPort=${rear_airy["DPort"]}&\
ReMode=${rear_airy["ReMode"]}&\
SynSrc=${rear_airy["SynSrc"]}&\
PDomNum=${rear_airy["PDomNum"]}&\
RTPdelay=${rear_airy["RTPdelay"]}&\
NoLeap=${rear_airy["NoLeap"]}&\
SyTOut=${rear_airy["SyTOut"]}&\
UToL=${rear_airy["UToL"]}&\
LToU=${rear_airy["LToU"]}&\
OpM=${rear_airy["OpM"]}&\
PhaLockSet=${rear_airy["PhaLockSet"]}&\
GPSBRate=${rear_airy["GPSBRate"]}&\
ImuCtrl=${rear_airy["ImuCtrl"]}&\
IPort=${rear_airy["IPort"]}&\
ImuOutR=${rear_airy["ImuOutR"]}&\
AccRg=${rear_airy["AccRg"]}&\
GyroRg=${rear_airy["GyroRg"]}&\
Lpf=${rear_airy["Lpf"]}&\
ResDefault=${rear_airy["ResDefault"]}&\
tabs-two=on&\
RefEn=${rear_airy["RefEn"]}&\
TFilLvl=${rear_airy["TFilLvl"]}&\
BlkRnDis=${rear_airy["BlkRnDis"]}&\
Rn=${rear_airy["Rn"]}&\
Blk=${rear_airy["Blk"]}&\
FSAgl=${rear_airy["FSAgl"]}&\
DZ10cm=${rear_airy["DZ10cm"]}&\
C81858993=${rear_airy["C81858993"]}&\
GapFill=${rear_airy["GapFill"]}&\
save_param_performance=Save&\
mode2=Mode2" \
http://$1/Parameter_Setting.html \
-o /dev/null || true
sleep 5
}


function try_set_front_airy_config() {
    local_ip=$1
    remote_ip=$2
    local config_changed=0
    #echo "开始配置前雷达"
    #echo "前雷达接口名称: $net_front" 
    nmcli con add con-name "front" ifname $net_front type ethernet ipv4.addresses $local_ip/24 ipv4.gateway 192.168.168.100 ipv4.method manual > /dev/null 2>&1
    nmcli con up front > /dev/null 2>&1
    get_airy_config $remote_ip
    if [ $? -ne 0 ]; then
    	nmcli con down front > /dev/null 2>&1
    	nmcli con delete front > /dev/null 2>&1
        echo "获取配置失败"
        return 1
    fi
    print_airy_config
    check_front_variables
    if [ $? -ne 0 ]; then
        set_front_airy_config $remote_ip
        #set_front_airy_config_performance $remote_ip
        nmcli connection modify front ipv4.addresses ${front_airy["DstIp"]}/24 > /dev/null 2>&1
	nmcli connection up front > /dev/null 2>&1
        get_airy_config ${front_airy["SrcIp"]}
        if [ $? -ne 0 ]; then
    	    nmcli con down front > /dev/null 2>&1
    	    nmcli con delete front > /dev/null 2>&1
            echo "获取配置失败"
            return 1
        fi
    	nmcli con down front > /dev/null 2>&1
    	nmcli con delete front > /dev/null 2>&1
        print_airy_config
        check_front_variables
        if [ $? -ne 0 ]; then
            echo "配置不匹配"
            return 1
        fi
            config_changed=1
    fi
        if [ $config_changed -eq 1 ]; then
            sleep 5
        fi
    nmcli con down front > /dev/null 2>&1
    nmcli con delete front > /dev/null 2>&1
    return 0
}

function try_set_rear_airy_config() {
    local_ip=$1
    remote_ip=$2
    local config_changed=0
    #echo "开始配置后雷达"
    #echo "后雷达接口名称: $net_rear" 
    nmcli con add con-name "rear" ifname $net_rear type ethernet ipv4.addresses $local_ip/24 ipv4.gateway 192.168.168.100 ipv4.method manual > /dev/null 2>&1
    nmcli con up rear > /dev/null 2>&1
    get_airy_config $remote_ip
    if [ $? -ne 0 ]; then
    	nmcli con down rear > /dev/null 2>&1
    	nmcli con delete rear > /dev/null 2>&1
        echo "获取配置失败"
        return 1
    fi
    print_airy_config
    check_rear_variables
    if [ $? -ne 0 ]; then
        set_rear_airy_config $remote_ip
        #set_rear_airy_config_performance $remote_ip
        nmcli connection modify rear ipv4.addresses ${rear_airy["DstIp"]}/24 > /dev/null 2>&1
	nmcli connection up rear > /dev/null 2>&1
	get_airy_config ${rear_airy["SrcIp"]}
        if [ $? -ne 0 ]; then
	    nmcli con down rear > /dev/null 2>&1
	    nmcli con delete rear > /dev/null 2>&1
            echo "获取配置失败"
            return 1
        fi
	nmcli con down rear > /dev/null 2>&1
	nmcli con delete rear > /dev/null 2>&1
        print_airy_config
        check_rear_variables
        if [ $? -ne 0 ]; then
            echo "配置不匹配"
            return 1
        fi
    fi
    if [ $config_changed -eq 1 ]; then
        sleep 5
    fi
    nmcli con down rear > /dev/null 2>&1
    nmcli con delete rear > /dev/null 2>&1
    return 0
}

auto_conf_front_airy() {
    # 如果网络和雷达参数本来就正确，直接跳过，不删不重建
    if is_front_config_ok; then
        echo "检测到前雷达配置及网络已正确，跳过前雷达重新配置"
        # 确保 front 连接存在且参数正确，以便后续根据需要直接启用
        if ! is_front_nm_ok; then
            if [ -n "$net_front" ]; then
                nmcli con delete front > /dev/null 2>&1
                nmcli con add con-name "front" ifname $net_front type ethernet ipv4.addresses ${front_airy["DstIp"]}/24 ipv4.gateway "" ipv4.method manual ipv4.never-default yes > /dev/null 2>&1
                nmcli connection down front > /dev/null 2>&1
            fi
        fi
        return 1
    fi

    # 配置前雷达前，临时关闭后雷达网口，避免两个网口同时在同一网段导致的 IP 冲突等问题
    if [ -n "$net_rear" ]; then
        ifconfig $net_rear down > /dev/null 2>&1
    fi

    # 否则清理旧连接后再进行自动配置
    if [ -n "$net_front" ]; then
        nmcli con delete front > /dev/null 2>&1
        clean_nm_connections $net_front
        ifconfig $net_front down > /dev/null 2>&1
    fi

    LOCAL_IP=192.168.1.102 
    REMOTE_IP=192.168.1.200
    echo "尝试配置前雷达，接口 $net_front 本地IP $LOCAL_IP 雷达IP $REMOTE_IP"
    try_set_front_airy_config $LOCAL_IP $REMOTE_IP
    if [ $? -ne 0 ]; then
        #该雷达可能用作过后雷达，尝试配置
        LOCAL_IP=192.168.2.102 
        REMOTE_IP=192.168.2.200
	echo "再次尝试配置前雷达，接口 $net_front 本地IP $LOCAL_IP 雷达IP $REMOTE_IP"
	try_set_front_airy_config $LOCAL_IP $REMOTE_IP
        if [ $? -ne 0 ]; then
            echo "前雷达配置失败"
            return 1
        fi
    fi
    echo "前雷达配置成功 ！！！"
    nmcli con add con-name "front" ifname $net_front type ethernet ipv4.addresses ${front_airy["DstIp"]}/24 ipv4.gateway 192.168.168.100 ipv4.method manual > /dev/null 2>&1
    nmcli connection down front > /dev/null 2>&1
    return 0
}

auto_conf_rear_airy() {
    # 如果网络和雷达参数本来就正确，直接跳过，不删不重建
    if is_rear_config_ok; then
        echo "检测到后雷达配置及网络已正确，跳过后雷达重新配置"
        # 确保 rear 连接存在且参数正确，以便后续根据需要直接启用
        if ! is_rear_nm_ok; then
            if [ -n "$net_rear" ]; then
                nmcli con delete rear > /dev/null 2>&1
                nmcli con add con-name "rear" ifname $net_rear type ethernet ipv4.addresses ${rear_airy["DstIp"]}/24 ipv4.gateway "" ipv4.method manual ipv4.never-default yes > /dev/null 2>&1
                nmcli connection down rear > /dev/null 2>&1
            fi
        fi
        return 1
    fi

    # 配置后雷达前，临时关闭前雷达网口，避免两个网口同时在同一网段导致的 IP 冲突等问题
    if [ -n "$net_front" ]; then
        ifconfig $net_front down > /dev/null 2>&1
    fi

    # 否则清理旧连接后再进行自动配置
    if [ -n "$net_rear" ]; then
        nmcli con delete rear > /dev/null 2>&1
        clean_nm_connections $net_rear
        ifconfig $net_rear down > /dev/null 2>&1
    fi

    # 假设后雷达已经按 rear_airy 配置过，优先按 192.168.2.x 网段尝试
    LOCAL_IP=192.168.2.102 
    REMOTE_IP=192.168.2.200
    echo "尝试配置后雷达，接口 $net_rear 本地IP $LOCAL_IP 雷达IP $REMOTE_IP"
    try_set_rear_airy_config $LOCAL_IP $REMOTE_IP
    if [ $? -ne 0 ]; then
        # 如果不是已配置的后雷达网段，尝试按 192.168.1.x 网段配置
        LOCAL_IP=192.168.1.102 
        REMOTE_IP=192.168.1.200
	echo "再次尝试配置后雷达，接口 $net_rear 本地IP $LOCAL_IP 雷达IP $REMOTE_IP"
        try_set_rear_airy_config $LOCAL_IP $REMOTE_IP
        if [ $? -ne 0 ]; then
            echo "后雷达配置失败"
            return 1
        fi
    fi
    echo "后雷达配置成功 ！！！"
    nmcli con add con-name "rear" ifname $net_rear type ethernet ipv4.addresses ${rear_airy["DstIp"]}/24 ipv4.gateway 192.168.168.100 ipv4.method manual > /dev/null 2>&1
    nmcli connection down rear > /dev/null 2>&1
    return 0
}

try_get_airy_data() {
    rm -f /tmp/udp.txt
    timeout 5 nc -u -l -p $1 -s $2 > /tmp/udp.txt
    if [ -s /tmp/udp.txt ]; then
        return 0
    else
        return 1
    fi
}

auto_conf_front_airy
FRONT_RES=$?
auto_conf_rear_airy
REAR_RES=$?

if [ $FRONT_RES -eq 0 ] || [ $FRONT_RES -eq 1 ]; then
    # 只有在 NetworkManager 中 front 连接参数不满足期望时才修改，避免重复操作
    if ! is_front_nm_ok; then
        echo "修改前雷达网络连接参数"
        nmcli connection modify front ipv4.gateway ""
        nmcli connection modify front ipv4.never-default yes
    fi
    # 如果 front 连接已经是激活状态，则无需再次 up，加快重复执行时的速度
    if ! nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "front"; then
        nmcli connection up front > /dev/null 2>&1
    fi
fi
if [ $REAR_RES -eq 0 ] || [ $REAR_RES -eq 1 ]; then
    # 只有在 NetworkManager 中 rear 连接参数不满足期望时才修改，避免重复操作
    if ! is_rear_nm_ok; then
        echo "修改后雷达网络连接参数"
        nmcli connection modify rear ipv4.gateway ""
        nmcli connection modify rear ipv4.never-default yes
    fi
    # 如果 rear 连接已经是激活状态，则无需再次 up，加快重复执行时的速度
    if ! nmcli -t -f NAME connection show --active 2>/dev/null | grep -qx "rear"; then
        nmcli connection up rear > /dev/null 2>&1
    fi
fi

if [ $FRONT_RES -le 1 ] && [ $REAR_RES -le 1 ]; then
    echo "恭喜所有雷达配置成功！！"
else
    echo "存在配置失败的雷达，建议检查网络接口状态，如果网络接口正常但是雷达IP无法访问，可以使用tcpdump -i <接口> -vvv 来尝试获取雷达的真实IP地址"
fi


echo -e "\n\n进行数据测试..."
if [ $FRONT_RES -eq 0 ]; then
    # 配置发生了改变，需要进行打流测试
    sleep 1
    try_get_airy_data ${front_airy["MPort"]} ${front_airy["DstIp"]}
    if [ $? -eq 0 ]; then
        echo "前雷达数据流可用，雷达正常..."
    else
    	echo "前雷达数据存在异常，请检查..."
    fi
elif [ $FRONT_RES -eq 1 ]; then
    # 配置未改变，跳过打流测试
    echo "前雷达配置未改变，跳过打流测试"
else
    echo "前雷达配置失败"
fi
if [ $REAR_RES -eq 0 ]; then
    # 配置发生了改变，需要进行打流测试
    sleep 1
    try_get_airy_data ${rear_airy["MPort"]} ${rear_airy["DstIp"]}
    if [ $? -eq 0 ]; then
        echo "后雷达数据流可用，雷达正常..."
    else
    	echo "后雷达数据存在异常，请检查..."
    fi
elif [ $REAR_RES -eq 1 ]; then
    # 配置未改变，跳过打流测试
    echo "后雷达配置未改变，跳过打流测试"
else
    echo "后雷达配置失败"
fi






