#!/bin/bash

logging=0
while getopts n:l: option
do

    case "${option}"
    in
	n)num_agents=${OPTARG};;
	l)logging=1;;
    esac
done

echo "[CASA-SIM] Starting CASA Simulation"
log_dir_name=casa_sim-$(date +%Y%m%d_%H%M%S)
mkdir -p ../log/$log_dir_name
echo "log: $logging"
for (( i=1; i<=$num_agents; i++ ))
do
    cd ~/PX4-Autopilot
    echo "[CASA-SIM] Starting instance $i"
    x=$((2*i))
    log_file_name=~/CASA/log/$log_dir_name/px4_$i.log
    echo $log_file_name
    export PX4_SYS_AUTOSTART=4001
    export PX4_GZ_MODEL_POSE="0,$x"
    export PX4_GZ_MODEL=x500
    if [ $logging == 1 ]; then
	screen -d -m -L -Logfile $log_file_name ./build/px4_sitl_default/bin/px4 -i $i
    else
	echo "[CASA-SIM] starting without logging"
	screen -d -m ./build/px4_sitl_default/bin/px4 -i $i
    fi
    
    if [ $i == 1 ]; then
	echo "[CASA-SIM] Waiting for GUI..."
	sleep 5
    else
	sleep 1
    fi
done
