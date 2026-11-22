#!/usr/bin/env bash
# Mide la camara OAK-D del TB4 por Wi-Fi (sin internet) y guarda un informe de texto.
# No mueve el robot. Uso (desde WSL, con el PC en el Wi-Fi del lab, SIN cable al robot):
#   ./medir_camara_wifi.sh [IP_ROBOT] [DOMAIN_ID]     # por defecto 192.168.0.101 67
# Informe: ~/medicion_camara_wifi.txt  (luego reconecta a internet y avisa a Claude)
IP="${1:-192.168.0.101}"; DOM="${2:-67}"
OUT="$HOME/medicion_camara_wifi.txt"
cd "$(dirname "$0")/.." || exit 1
source /opt/ros/jazzy/setup.bash
source ./tb4_connect.sh "$IP" "$DOM" peers || exit 1
T=/oakd/rgb/preview/image_raw/compressed
{
  echo "== $(date) | robot $IP dominio $DOM | modo ${TB4_MODE:-?}"
  echo "-- ping"; ping -c 5 -W 1 "$IP" | tail -3
  echo "-- start_camera"; timeout 20 ros2 service call /oakd/start_camera std_srvs/srv/Trigger | tail -1
  sleep 4
  for i in 1 2; do
    echo "-- $T (pasada $i): hz"; timeout 12 ros2 topic hz "$T" --window 30 2>&1 | tail -3
    echo "-- bw";               timeout 10 ros2 topic bw "$T" 2>&1 | tail -2
  done
  echo "-- /scan hz (referencia)"; timeout 10 ros2 topic hz /scan --window 20 2>&1 | tail -2
} 2>&1 | tee "$OUT"
echo; echo "Guardado en $OUT"
