#!/bin/bash

for pid in $(find /proc/*/fd -type l 2>/dev/null | grep -s inotify | awk -F/ '{print $3}' | sort -u); do
  echo "PID: $pid, Process: $(ps -p $pid -o comm=), Inotify Count: $(cat /proc/$pid/limits | grep "inotify")"
done