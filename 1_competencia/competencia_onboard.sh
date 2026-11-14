#!/usr/bin/env bash
# Lanzador unificado a bordo de la Raspberry Pi (Edge Execution):
#   ./competencia_onboard.sh               # Tarea A guiada por senales + LiDAR (sin --goal)
#   ./competencia_onboard.sh --goal 3.0 0  # Tarea A con meta conocida
cd "$(dirname "$0")" || exit 1

export OPENBLAS_NUM_THREADS=1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=67
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Alinear reloj del Create 3 con la Pi antes de arrancar (sin tocar /start_motor del RPLIDAR)
curl -s -X POST http://192.168.186.2/api/restart-ntpd > /dev/null 2>&1 || true

echo "[ON-BOARD] Ejecutando mision en Raspberry Pi (OPENBLAS_NUM_THREADS=1)..."
exec python3 mision.py --tarea A "$@"
