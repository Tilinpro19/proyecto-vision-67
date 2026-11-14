#!/usr/bin/env bash
# Lanzador unificado de competencia desde WSL (Tarea A con o sin --goal, o Tarea B):
#   ./competencia.sh A                     # Tarea A guiada por senales + LiDAR (sin --goal)
#   ./competencia.sh A --goal 3.0 0.5      # Tarea A por coordenadas
#   ./competencia.sh B                     # Tarea B por gestos
cd "$(dirname "$0")" || exit 1
TAREA="${1:-A}"
shift 2>/dev/null || true

export OPENBLAS_NUM_THREADS=1
source /opt/ros/jazzy/setup.bash
if [ -z "$TB4_IP" ]; then
  if [ -z "$ROBOT_IP" ] && [ -f ~/.tb4_robot ]; then
    source ~/.tb4_robot
  else
    source ../tb4_connect.sh "${ROBOT_IP:-192.168.185.3}" "${ROBOT_ID:-67}" || exit 1
  fi
fi

exec python3 mision.py --tarea "$TAREA" "$@"
