#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=99
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

python3 ~/turtleclaude4/proyecto_final/2_laboratorio/robot_simulado.py --circuito obstaculo > /dev/null 2>&1 &
SIM_PID=$!

trap "kill $SIM_PID 2>/dev/null" EXIT

sleep 2

cd ~/turtleclaude4/proyecto_final/1_competencia
python3 navegacion.py --goal 2.5 0.0 --pausa 1.0
