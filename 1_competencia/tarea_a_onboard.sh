#!/usr/bin/env bash
# ==============================================================================
# Tarea A On-Board: Ejecuta la mision de navegacion A -> B -> A directamente
# en la Raspberry Pi 4 con 0 ms de latencia y sin dependencia de red.
# ==============================================================================
cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=67

# Sincronizar reloj de Create 3 preventivamente
curl -s -X POST http://192.168.186.2/api/restart-ntpd > /dev/null 2>&1 || true

echo "[ON-BOARD] Ejecutando Tarea A directamente en Raspberry Pi..."
exec python3 mision.py --tarea A "$@"
