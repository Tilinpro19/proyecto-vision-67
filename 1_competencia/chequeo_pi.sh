#!/usr/bin/env bash
# ==============================================================================
# Chequeo de SOLO LECTURA en la Raspberry Pi antes de correr la mision a bordo.
# No mueve el robot, no publica nada, no instala ni modifica archivos.
#
#   cd ~/turtleclaude4/1_competencia && ./chequeo_pi.sh
# ==============================================================================
cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-67}
ok()   { printf '  [OK]    %s\n' "$*"; }
mal()  { printf '  [FALLA] %s\n' "$*"; FALLO=1; }
FALLO=0

echo "== Hardware / carga =="
echo "  nucleos: $(nproc) | $(uptime | sed 's/.*load average/load average/')"
command -v vcgencmd >/dev/null && echo "  throttled: $(vcgencmd get_throttled) | $(vcgencmd measure_temp)"
echo "  hora de la Pi: $(date '+%F %T')"

echo "== Python / OpenCV / modelos =="
python3 - <<'EOF' || FALLO=1
import os, sys
try:
    import cv2, numpy
    print(f"  [OK]    cv2 {cv2.__version__}, numpy {numpy.__version__}")
except Exception as e:
    print(f"  [FALLA] cv2/numpy: {e}"); sys.exit(1)
for m in ("face_detection_yunet_2023mar.onnx", "face_recognition_sface_2021dec.onnx"):
    p = os.path.join("modelos", m)
    print(f"  [OK]    {p}" if os.path.isfile(p) else f"  [FALLA] falta {p}")
try:
    import facial
    facial.Rostros().calentar()
    print("  [OK]    facial.Rostros() carga y calienta")
except BaseException as e:
    print(f"  [FALLA] facial.Rostros(): {e}"); sys.exit(1)
EOF
n=$(ls galeria/*.jpg galeria/*.jpeg galeria/*.png galeria/*/* 2>/dev/null | wc -l)
[ "$n" -gt 0 ] && ok "galeria: $n imagen(es)" || mal "galeria vacia (copia las fotos reales a galeria/)"

echo "== Imports de la mision (sin ROS en marcha) =="
python3 -c "import mision, navegacion, nav_core" 2>&1 | tail -2 && ok "mision.py importa" || mal "mision.py no importa"

echo "== Topics del robot (domain $ROS_DOMAIN_ID) =="
T=$(timeout 15 ros2 topic list 2>/dev/null)
for t in /scan /odom /cmd_vel; do
  echo "$T" | grep -qx "$t" && ok "$t" || mal "no aparece $t"
done
echo "$T" | grep -q "oakd/rgb/preview/image_raw/compressed" && ok "camara OAK-D" || mal "no aparece la camara (oakd/rgb/preview/image_raw/compressed)"

echo
[ "$FALLO" = 0 ] && echo "Todo listo para: ./tarea_a_onboard.sh --goal X Y --dry" || echo "Hay FALLAS arriba: resuelvelas antes de correr la mision."
exit $FALLO
