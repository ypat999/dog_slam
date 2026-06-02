#!/bin/bash

#check airy lidars are configured
/opt/runtime/bin/config_airy.sh

#wait time sync ok
/opt/runtime/bin/robot_wait_time_sync.sh

#fix calibration_results.yaml`s topic names`
/opt/runtime/bin/fix_calibration_topic_name.sh /ota/calibration_results.yaml

exit 0
