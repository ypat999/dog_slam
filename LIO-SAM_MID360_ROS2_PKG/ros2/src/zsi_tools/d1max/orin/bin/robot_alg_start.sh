#!/bin/sh
. /opt/runtime/env.bash

echo "1" | sudo -S chown -R jszr:jszr /tmp

bash /opt/runtime/bin/alg-manager.sh &

while true; do
  sleep 3
done



