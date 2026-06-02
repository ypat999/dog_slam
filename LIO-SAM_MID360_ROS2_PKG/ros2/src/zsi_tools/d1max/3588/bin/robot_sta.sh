#!/bin/bash

STA_DEVICE="wlan0"
WIFI_CONFIG_FILE=/userdata/config/wifi_config.bash

PROGRAM_NAME=$(basename $0)
usage() {
    echo "usage: $PROGRAM_NAME status"
    echo "usage: $PROGRAM_NAME scan"
    echo "usage: $PROGRAM_NAME stop"
    echo "usage: $PROGRAM_NAME start"
    echo "usage: $PROGRAM_NAME save"
    echo "usage: $PROGRAM_NAME enable"
    echo "usage: $PROGRAM_NAME disable"
    echo "usage: $PROGRAM_NAME connect <ssid> <password>"
}

SERVICE_NAME="wpa_supplicant"
CONFIG_FILE=/userdata/config/sta.conf
SERVICE_FILE=/etc/systemd/system/wpa_supplicant.service

if [ ! -f $CONFIG_FILE ]; then
cat > $CONFIG_FILE << EOF
ctrl_interface=/var/run/wpa_supplicant
update_config=1

network={
    ssid="RobotNet"
    bssid=80:ae:54:9e:78:2c
    psk="........"
    mesh_fwding=1
    disabled=1
}

EOF

cat > $SERVICE_FILE << EOF
[Unit]
Description=WPA supplicant daemon (Wi-Fi connection)
After=network.target dbus.service
Wants=network.target

[Service]
Type=simple
ExecStart=/sbin/wpa_supplicant -c $CONFIG_FILE -i wlan0
ExecReload=/bin/kill -HUP $MAINPID
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
systemctl daemon-reload
systemctl restart wpa_supplicant.service

fi

# 查找最佳BSSID的核心函数
# 用法: find_best_bssid <SSID名称>
# 返回: 最佳BSSID (格式: "BSSID:XX:XX:XX:XX:XX:XX,频率:XXXX,信号强度:XX")
find_best_bssid() {
    local target_ssid="$1"
    local interface="${2:-wlan0}"  # 可选参数：无线接口，默认为wlan0
    
    # 输入验证
    if [ -z "$target_ssid" ]; then
        echo "错误: 必须提供SSID名称作为参数" >&2
        return 1
    fi
    
    echo "正在扫描SSID: $target_ssid (接口: $interface)..." >&2
    
    # 执行扫描
    if ! wpa_cli -i "$interface" scan > /dev/null 2>&1; then
        echo "错误: 扫描失败，请检查接口 $interface 是否正确" >&2
        return 2
    fi
    
    # 等待扫描完成
    sleep 5
    
    # 获取扫描结果并处理
    local best_bssid=""
    local best_frequency=0
    local best_signal=-1000
    local best_is_5ghz=0
    
    # 使用awk处理扫描结果，应用选择规则
    wpa_cli -i "$interface" scan_results | awk -F'\t' -v ssid="$target_ssid" '
    NR < 2 { next }  # 跳过标题行
    $5 == ssid {  # 只处理匹配目标SSID的行
        bssid = $1
        frequency = $2
        signal_level = $3
        flags = $4
        
        # 判断是否为5GHz频段
        is_5ghz = (frequency > 5000) ? 1 : 0
        # 只保留5G ??
        #if (is_5ghz == 0)
        #    next

        # 应用选择规则：优先5GHz，然后信号强度
        if (best_bssid == "") {
            # 第一个候选
            best_bssid = bssid
            best_frequency = frequency
            best_signal = signal_level
            best_is_5ghz = is_5ghz
        } else {
            if (is_5ghz && !best_is_5ghz) {
                # 当前是5G而最佳不是，优先选择5G
                best_bssid = bssid
                best_frequency = frequency
                best_signal = signal_level
                best_is_5ghz = is_5ghz
            } else if (is_5ghz == best_is_5ghz) {
                # 相同频段，选择信号更强的
                if (signal_level > best_signal) {
                    best_bssid = bssid
                    best_frequency = frequency
                    best_signal = signal_level
                    best_is_5ghz = is_5ghz
                }
            }
            # 如果当前是2.4G而最佳是5G，则保持最佳不变
        }
    }
    END {
        if (best_bssid != "") {
            printf "BSSID:%s,频率:%d,信号强度:%d", best_bssid, best_frequency, best_signal
        } else {
            print "未找到匹配的网络"
            exit 3
        }
    }'
}

if [ $# -eq 1 ]; then
    case $1 in
        "scan")
		wpa_cli -i $STA_DEVICE scan
		sleep 5
		wpa_cli -i $STA_DEVICE scan_results
		exit $?
		;;
        "save")
		wpa_cli -i $STA_DEVICE save_config 0
		;;
	"start")
    		wpa_cli -i $STA_DEVICE enable_network 0
		;;
	"stop")
    		wpa_cli -i $STA_DEVICE disable_network 0
		;;
        "enable")
		sed -i -e "s/^STA_ENABLE=.*/STA_ENABLE=1/" $WIFI_CONFIG_FILE
		;;
        "disable")
		sed -i -e "s/^STA_ENABLE=.*/STA_ENABLE=0/" $WIFI_CONFIG_FILE
		;;
        "status")
        	echo "WPA info:"
    		wpa_cli -i $STA_DEVICE status 0
    		echo "Driver info:"
    		cat /proc/net/rtl8852be/$STA_DEVICE/ap_info
		;;
	*) echo "cmd $1 invalid !!"
		exit 5;
		usage
		;;
    esac
    exit 0;
elif [ $# -eq 3 ]; then
    case $1 in
        "connect")
		result=$(find_best_bssid "$2" "$STA_DEVICE")
		if [ $? -ne 0 ]; then
		    echo "未找到匹配的 $2 可以连接"
		    exit 6;
		    usage
		fi
		bssid=$(echo "$result" | sed 's/.*BSSID:\([^,]*\),.*/\1/')
                freq=$(echo "$result" | sed 's/.*频率:\([^,]*\),.*/\1/')
                signal=$(echo "$result" | sed 's/.*信号强度:\([^,]*\).*/\1/')
                echo "解析结果:"
                echo "  BSSID: $bssid"
                echo "  频段: $freq MHz"
                echo "  信号强度: $signal dBm"
        	# 只保留5G ??
		#if [ $freq -lt 5000 ]; then
		#    echo "目前暂时只支持5G频段的STA"
		#    exit 7
	        #fi
		wpa_cli -i $STA_DEVICE set_network 0 ssid '"'"$2"'"'
		wpa_cli -i $STA_DEVICE set_network 0 bssid $bssid
		wpa_cli -i $STA_DEVICE set_network 0 psk '"'"$3"'"'
		wpa_cli -i $STA_DEVICE enable_network 0
		#wpa_cli -i $STA_DEVICE save_config
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


