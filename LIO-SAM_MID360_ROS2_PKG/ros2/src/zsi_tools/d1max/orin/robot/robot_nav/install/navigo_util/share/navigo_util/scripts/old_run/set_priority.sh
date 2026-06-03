#!/bin/bash

PROCESS_NAME=$1

PIDS=$(pidof "$PROCESS_NAME")

if [ -z "$PIDS" ]; then
  echo "Error: Process '$PROCESS_NAME' not found!"
  exit 1
fi

if [ $(echo "$PIDS" | wc -w) -eq 1 ]; then
  PID=$PIDS
else
  echo "Found the following PIDs for '$PROCESS_NAME':"
  echo "$PIDS"
  read -p "Enter the PID to set priority: " PID

  if ! echo "$PIDS" | grep -wq "$PID"; then
    echo "Error: Invalid PID entered!"
    exit 1
  fi
fi

echo 1 | sudo -S chrt -p --rr 90 $PID
if [ $? -eq 0 ]; then
  echo "Successfully set '$PROCESS_NAME' to high priority."
else
  echo "Failed to set priority for process '$PROCESS_NAME'."
  exit 1
fi
