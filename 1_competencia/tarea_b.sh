#!/usr/bin/env bash
# Tarea B en un solo comando (desde WSL, dentro de 1_competencia/):
#   ./tarea_b.sh                     # webcam 0; palma = armar, V = escanear / anunciar
#   ./tarea_b.sh --cam 1 --lineal 0.2 --clahe
# Robot distinto: ROBOT_IP=192.168.0.105 ROBOT_ID=68 ./tarea_b.sh ...
cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash
if [ -z "$TB4_IP" ]; then
  if [ -z "$ROBOT_IP" ] && [ -f ~/.tb4_robot ]; then
    source ~/.tb4_robot
  else
    source ../tb4_connect.sh "${ROBOT_IP:-192.168.0.104}" "${ROBOT_ID:-67}" || exit 1
  fi
fi
exec python3 mision.py --tarea B "$@"
