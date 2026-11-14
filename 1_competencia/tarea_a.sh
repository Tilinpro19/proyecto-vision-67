#!/usr/bin/env bash
# Tarea A en un solo comando (desde WSL, dentro de 1_competencia/):
#   ./tarea_a.sh --goal 3.0 0.5            # B a 3 m al frente y 0.5 m a la izquierda de A
#   ./tarea_a.sh --goal 3.0 0.5 --param max_linear=0.25 --ver
# Robot distinto: ROBOT_IP=192.168.0.105 ROBOT_ID=68 ./tarea_a.sh ...
cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash
if [ -z "$TB4_IP" ]; then
  if [ -z "$ROBOT_IP" ] && [ -f ~/.tb4_robot ]; then
    source ~/.tb4_robot
  else
    _TB4_CONN="../tb4_connect.sh"
    [ -f "$_TB4_CONN" ] || _TB4_CONN="../../tb4_connect.sh"
    source "$_TB4_CONN" "${ROBOT_IP:-192.168.0.104}" "${ROBOT_ID:-67}" || exit 1
  fi
fi
exec python3 mision.py --tarea A "$@"
