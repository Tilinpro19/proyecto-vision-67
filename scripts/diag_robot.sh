#!/usr/bin/env bash
# Corre DENTRO DEL ROBOT (por SSH, en su propia sesion tmux) mientras pruebas la navegacion.
# Registra lo que la laptop no puede ver: CPU/memoria/temperatura/throttling de la Pi,
# procesos que mas consumen, hora, y el estado del Create 3 (dock, ruedas, sensores, bateria).
#
# Uso:
#   ./diag_robot.sh            # Ctrl+C para terminar
#   ./diag_robot.sh 5          # intervalo en segundos (por defecto 2)
#
# Salida: ~/diag_robot/<fecha>/  sistema.csv  create3.log  nodos.log  resumen.txt
# Solo lee: no cambia nada del robot. Es liviano, pero cada snapshot de Create 3 arranca ros2 (~1-2 s de CPU).

INTERVAL="${1:-2}"
OUT="$HOME/diag_robot/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
[ -f /etc/turtlebot4/setup.bash ] && source /etc/turtlebot4/setup.bash
source /opt/ros/jazzy/setup.bash

echo "t_s,hora_robot,load1,load5,cpu_pct,mem_usada_pct,temp_c,throttled,top_proc" > "$OUT/sistema.csv"
T0=$(date +%s)
LAST_SNAP=-100
LAST_NODES=-100

cpu_pct() {
  # % de CPU en uso en el intervalo (lectura de /proc/stat dos veces)
  read -r _ u1 n1 s1 i1 w1 q1 sq1 _ < /proc/stat; sleep 1
  read -r _ u2 n2 s2 i2 w2 q2 sq2 _ < /proc/stat
  local idle=$(( (i2 + w2) - (i1 + w1) ))
  local tot=$(( (u2 + n2 + s2 + i2 + w2 + q2 + sq2) - (u1 + n1 + s1 + i1 + w1 + q1 + sq1) ))
  [ "$tot" -gt 0 ] && echo $(( 100 * (tot - idle) / tot )) || echo 0
}

snap_create3() {
  {
    echo "=== $(date '+%F %T') (t=$(( $(date +%s) - T0 )) s) ==="
    for t in dock_status wheel_status hazard_detection stop_status battery_state; do
      echo "--- /$t"
      timeout 4 ros2 topic echo --once "/$t" 2>&1 | head -25
    done
  } >> "$OUT/create3.log"
}

snap_nodes() {
  {
    echo "=== $(date '+%F %T') ==="
    ros2 node list 2>&1 | sort
    for n in controller_server planner_server bt_navigator behavior_server smoother_server \
             velocity_smoother collision_monitor waypoint_follower slam_toolbox; do
      printf '%s: ' "$n"; timeout 3 ros2 lifecycle get "/$n" 2>&1 | head -1
    done
    echo "--- hz /scan (3 s)"; timeout 5 ros2 topic hz /scan 2>&1 | tail -2
    echo "--- hz /odom (3 s)"; timeout 5 ros2 topic hz /odom 2>&1 | tail -2
  } >> "$OUT/nodos.log"
}

finish() {
  echo; echo "Resumen..."
  {
    echo "=== RESUMEN ROBOT ($(date '+%F %T')) ==="
    awk -F, 'NR>1 { n++; c+=$5; if ($5>cmax) cmax=$5; if ($7>tmax) tmax=$7;
                    if ($3>lmax) lmax=$3; if ($5>=90) sat++; if ($8!="0x0") thr[$8]++ }
             END { if (n) { printf "CPU media %.0f%% | max %d%% | muestras >=90%%: %d de %d\n", c/n, cmax, sat, n;
                            printf "Load1 max %.2f (4 nucleos: >4 = saturada)\n", lmax;
                            printf "Temp max %.1f C\n", tmax;
                            for (k in thr) printf "throttled=%s en %d muestras\n", k, thr[k] } }' "$OUT/sistema.csv"
    echo "--- Conclusiones"
    awk -F, 'NR>1 { n++; c+=$5; if ($7>tmax) tmax=$7; if ($5>=90) sat++; if ($8!="0x0") thr++ }
             END { if (!n) exit;
                   if (c/n>=75) print "- CPU media alta: Pi cerca de saturarse (SLAM+Nav2 juntos?)";
                   if (sat/n>0.2) print "- CPU >=90% en mas del 20% del tiempo: causa probable de timeouts/abortos de Nav2";
                   if (tmax>=75) print "- Temperatura alta (>=75C): la Pi puede reducir frecuencia";
                   if (thr>0) print "- La Pi reporto throttling/subvoltaje (throttled != 0x0): revisa bateria/alimentacion y temperatura";
                   if (c/n<75 && sat/n<=0.2 && tmax<75 && thr==0) print "- CPU/temperatura de la Pi normales: el problema NO es la carga de la Pi" }' "$OUT/sistema.csv"
    echo "--- Create 3 (ultimo snapshot)"
    tail -60 "$OUT/create3.log"
  } > "$OUT/resumen.txt"
  cat "$OUT/resumen.txt"
  echo; echo "Logs en $OUT"
  echo "Bajalos a la laptop:  scp -r ubuntu@<IP_ROBOT>:$OUT ~/"
  exit 0
}
trap finish INT TERM

echo "Registrando en $OUT (intervalo ${INTERVAL}s). Ctrl+C para terminar."
while true; do
  now=$(( $(date +%s) - T0 ))
  cpu=$(cpu_pct)
  load=$(cut -d' ' -f1,2 /proc/loadavg | tr ' ' ',')
  mem=$(free | awk '/Mem:/ { printf "%d", 100*$3/$2 }')
  temp=$(vcgencmd measure_temp 2>/dev/null | sed "s/[^0-9.]//g"); temp="${temp:-0}"
  thr=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2); thr="${thr:-0x0}"
  top=$(COLUMNS=200 ps -eo comm,pcpu --sort=-pcpu | sed -n '2,4p' | awk '{printf "%s:%s;", $1, $2}')
  echo "$now,$(date +%T),$load,$cpu,$mem,$temp,$thr,$top" >> "$OUT/sistema.csv"
  printf '[%4ds] CPU %3s%% | load %s | mem %s%% | %s C | thr %s | %s\n' "$now" "$cpu" "$load" "$mem" "$temp" "$thr" "$top"

  if [ $(( now - LAST_SNAP )) -ge 10 ]; then snap_create3; LAST_SNAP=$now; fi
  if [ $(( now - LAST_NODES )) -ge 60 ]; then snap_nodes; LAST_NODES=$now; fi
  sleep "$INTERVAL"
done
