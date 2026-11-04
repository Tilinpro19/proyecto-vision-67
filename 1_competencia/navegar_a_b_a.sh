#!/usr/bin/env bash
# ==============================================================================
# NAVEGACION AUTONOMA PURA: A -> B -> A (TurtleBot 4 Lite)
# Sin camara, sin reconocimiento facial, sin dependencias de vision.
#
# Uso:
#   ./navegar_a_b_a.sh 1.5 0.0              # Avanza 1.5 m al frente y regresa a A
#   ./navegar_a_b_a.sh 2.0 0.5              # B en (x=2.0 m, y=0.5 m) y regresa a A
#   ./navegar_a_b_a.sh 1.0 0.0 --dry        # Modo en seco: lee sensores sin mover ruedas
# ==============================================================================

cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=67

# Sincronizar reloj de Create 3 preventivamente
curl -s -X POST http://192.168.186.2/api/restart-ntpd > /dev/null 2>&1 || true

GOAL_X="${1:-1.0}"
GOAL_Y="${2:-0.0}"
shift 2 2>/dev/null || true

echo "=========================================================="
echo " INICIANDO NAVEGACION AUTONOMA A -> B -> A"
echo " Meta B: (x=$GOAL_X m, y=$GOAL_Y m)"
echo " Modo: VFH-lite + SpatialKDTree2D + BreadcrumbGraph + ICP"
echo "=========================================================="

exec python3 navegacion.py --goal "$GOAL_X" "$GOAL_Y" --pausa 2.0 "$@"
