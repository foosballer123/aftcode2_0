#!/bin/bash

countdown() {
  local SECONDS_LEFT=$1
  local START_TIME=$(date +%s)
  local END_TIME=$((START_TIME + SECONDS_LEFT))

  while [[ $(date +%s) -lt $END_TIME ]]; do
    local TIME_NOW=$(date +%s)
    local REMAINING=$((END_TIME - TIME_NOW))
    
    # Format remaining seconds into HH:MM:SS format using the date command's -d flag.
    printf "\rTime remaining: %s" "$(date -u -d "@$REMAINING" +%H:%M:%S)"
    sleep 0.1 # Sleep for a fraction of a second to reduce CPU usage.
  done

  echo -e "\rTime remaining: 00:00:00\nTime's up!"
}
