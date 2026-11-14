#!/usr/bin/env bash
# Configura ROS para el robot a partir de TB4_IP / ROS_DOMAIN_ID / TB4_MODE.
#   TB4_MODE=multicast  -> descubrimiento normal (Ubuntu nativo, misma red)
#   TB4_MODE=peers      -> unicast con ROS_STATIC_PEERS=TB4_IP + perfil de 100 peers (WSL / redes sin multicast)
#   TB4_MODE=auto       -> peers si hay TB4_IP, si no multicast (por defecto)
source /opt/ros/jazzy/setup.bash
: "${ROS_DOMAIN_ID:=0}"
mode="${TB4_MODE:-auto}"
[ "$mode" = auto ] && { [ -n "$TB4_IP" ] && mode=peers || mode=multicast; }
{
  echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
  if [ "$mode" = peers ] && [ -n "$TB4_IP" ]; then
    echo "export ROS_STATIC_PEERS=$TB4_IP"
    if [ -f /ws/fastdds_pc.xml ]; then
      echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/ws/fastdds_pc.xml"
    else
      echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tb4/fastdds_pc.xml"
    fi
  fi
} > /tmp/tb4_env
source /tmp/tb4_env
echo "[tb4] ROS_DOMAIN_ID=$ROS_DOMAIN_ID | modo=$mode | robot=${TB4_IP:-?}" >&2
exec "$@"
