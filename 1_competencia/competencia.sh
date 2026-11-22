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
[ -f "$HOME/ros_extra/env.sh" ] && source "$HOME/ros_extra/env.sh"   # FFMPEGPacket para --camara-hd
if [ -z "$TB4_IP" ]; then
  if [ -z "$ROBOT_IP" ] && [ -f ~/.tb4_robot ]; then
    source ~/.tb4_robot
  else
    _TB4_CONN="../tb4_connect.sh"
    [ -f "$_TB4_CONN" ] || _TB4_CONN="../../tb4_connect.sh"
    source "$_TB4_CONN" "${ROBOT_IP:-192.168.0.101}" "${ROBOT_ID:-67}" peers || exit 1
  fi
fi
[ "$TB4_MODE" = "multicast" ] && echo "[AVISO] conectado en multicast (lento por Wi-Fi); reconecta con: source ../tb4_connect.sh $TB4_IP ${ROS_DOMAIN_ID:-67} peers"

exec python3 mision.py --tarea "$TAREA" "$@"
