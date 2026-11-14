#!/usr/bin/env bash
# ==============================================================================
# PARADA DE EMERGENCIA INSTANTANEA PARA TURTLEBOT 4
# Detiene la mision activa y frena los motores de inmediato.
# ==============================================================================
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-67}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "[PARADA] Enviando orden de parada de emergencia al robot (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)..."

# 1. Enviar senal de parada a la maquina de estados de mision.py
ros2 topic pub --once /mision/parar std_msgs/msg/Empty "{}" >/dev/null 2>&1 &

# 2. Frenar inmediatamente los motores del Create 3 (TwistStamped y Twist)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" >/dev/null 2>&1 &
ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 &

wait
echo "[OK] Robot detenido con exito."
