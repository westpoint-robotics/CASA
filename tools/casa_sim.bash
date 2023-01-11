#!/bin/bash

while getopts n:l: option
do

    case "${option}"
    in
	n)num_agents=${OPTARG};;
	l)logging=${OPTARG};;
    esac
done

echo "[CASA-SIM] Starting CASA Simulation"

for (( i=1; i<=$num_agents; i++ ))
do
    echo "[CASA-SIM] Starting instance $i"
    x=$((2*i))
    export PX4_SYS_AUTOSTART=4001
    export PX4_GZ_MODEL_POSE="0,$x"
    export PX4_GZ_MODEL=x500
    screen -d -m ./build/px4_sitl_default/bin/px4 -i $i
    if [ $i == 1 ]; then
	echo "[CASA-SIM] Waiting for GUI..."
	sleep 5
    else
	sleep 1
    fi
done
