#!/bin/bash 
###
 # @Author: richie.li
 # @Date: 2025-11-07 22:00:58
 # @LastEditors: richie.li
 # @LastEditTime: 2025-12-18 17:54:26
### 

source /opt/runtime/env.bash

FILE_PATH=/userdata/bak/calibration/

if [ -d $FILE_PATH ]; then
  echo "$FILE_PATH 目录存在, 删除中..."
  rm -rf $FILE_PATH
fi

mkdir -p $FILE_PATH

pose_calib_test 1

if [ -f $FILE_PATH/pose_calibrate_param.yaml ]; then
  echo "整机标定成功"
  exit 0
else
  echo "整机标定失败，请检查标定过程是否成功。"
  exit 1
fi
