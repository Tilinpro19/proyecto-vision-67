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
try:
    import facial
    print(f"  [OK]    detector {os.path.basename(facial.YUNET)} (OpenCV {cv2.__version__})")
    r = facial.Rostros()
    r.calentar()
    # antes solo se "calentaba" (y se tragaba el error): con OpenCV 4.6 + YuNet 2023mar no detectaba
    # NINGUNA cara y el chequeo decia OK. Ahora tiene que encontrar una cara real de la galeria.
    import glob
    fotos = sorted(glob.glob("galeria/*/*.jpg") + glob.glob("galeria/*/*.png"))
    if fotos:
        ok_caras = sum(1 for f in fotos[:5] if len(r.detectar(cv2.imread(f))[1]))
        if ok_caras == 0:
            print(f"  [FALLA] el detector no encuentra caras en {fotos[0]} (y otras 4)"); sys.exit(1)
        print(f"  [OK]    detecta caras: {ok_caras}/{min(5, len(fotos))} fotos de la galeria")
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
