#!/bin/bash

if [ "$1" == "-h" ] ; then
	echo "Usage ./perf.sh <filename> <interval> pid1 pid2..."
	exit 0
fi

# create a string with PID numbers with ',' between them
PIDS=""
for pid in "${@:3}"
do 
	PIDS=$PIDS","$pid
done

PIDS=${PIDS:1} # remove first ',' character

while true; do 
	top -b -n 1 -p$PIDS | awk 'NR > 7 { cpu += $9 } END {print cpu}' >> ${1}; 
	sleep ${2}; 
done



