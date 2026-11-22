#!/usr/bin/env bash
# Configura la terminal actual para hablar con un TurtleBot 4 (ROS 2 Jazzy).
# Prueba el descubrimiento DDS y, si la red bloquea multicast, pasa solo a
# ROS_STATIC_PEERS. Guarda el resultado en ~/.tb4_robot para reutilizarlo.
#
# Uso (siempre con "source", para que las variables queden en tu terminal):
#   source tb4_connect.sh <IP_ROBOT> <DOMAIN_ID> [auto|multicast|peers]
#   source tb4_connect.sh                # reconecta con el ultimo robot guardado
#   source tb4_connect.sh --off          # limpia la configuracion de esta terminal
#   source tb4_connect.sh --hora         # iguala el reloj del robot con el de la laptop (SSH + sudo)
#
# Al conectar, mide el desfase de reloj robot-laptop y, si pasa de 2 s, ofrece sincronizarlo
# (el robot sin internet no tiene NTP). Para omitirlo: TB4_NOSYNC=1 source tb4_connect.sh ...
# Usuario SSH del robot por defecto: ubuntu (cambia con TB4_USER).
#
# Para que cada terminal nueva arranque ya configurada (sin repetir pruebas):
#   echo '[ -f ~/.tb4_robot ] && source ~/.tb4_robot' >> ~/.bashrc

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  echo "Ejecutalo con source:  source $0 <IP_ROBOT> <DOMAIN_ID>"
  exit 1
fi

_tb4_key_topics() {
  # Cuenta cuantos topicos clave del robot se ven (0-4).
  ros2 topic list --no-daemon --spin-time "${TB4_SPIN:-8}" 2>/dev/null \
    | grep -cE '^/(scan|odom|cmd_vel|oakd/rgb/preview/image_raw)$'
}

_tb4_clock_offset() {
  # Segundos de desfase laptop - robot, medidos con el sello de /scan (reloj de la Pi; /odom si no hay /scan) ("none" si no llega).
  timeout 25 python3 - <<'PY' 2>/dev/null
import time, rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node('tb4_clock'); off = {}
lag = lambda m: time.time() - (m.header.stamp.sec + m.header.stamp.nanosec / 1e9)
n.create_subscription(LaserScan, '/scan', lambda m: off.setdefault('scan', lag(m)), qos_profile_sensor_data)
n.create_subscription(Odometry, '/odom', lambda m: off.setdefault('odom', lag(m)), qos_profile_sensor_data)
t = time.time()
while 'scan' not in off and time.time() - t < 15:
    rclpy.spin_once(n, timeout_sec=0.2)
v = off.get('scan', off.get('odom'))
print(v if v is not None else 'none')
PY
}

_tb4_sync_clock() {
  # Pone la hora de esta laptop en el robot por SSH y reinicia el bringup. Pide las contrasenas del robot.
  local ip="$1" user="${TB4_USER:-ubuntu}" sock="/tmp/tb4_ssh_$$"
  echo "== Sincronizando reloj de $user@$ip (pide la contrasena del robot; SSH una vez y sudo una vez)"
  if ! ssh -fN -M -S "$sock" -o ControlPersist=120 "$user@$ip"; then
    echo "[FALLA] no pude abrir SSH a $user@$ip"; return 1
  fi
  # La clave de sudo se lee aqui, una vez, y viaja por stdin (no por argumentos). El comando
  # va en UN solo sudo y la hora se toma justo al enviarlo, asi no cuenta lo que tardas en teclear.
  local pw rc
  read -rs -p "Contrasena de sudo en el robot (suele ser la misma): " pw; echo
  printf '%s\n' "$pw" | ssh -S "$sock" "$user@$ip" \
    "sudo -S -p '' bash -c 'date -u -s @$(date +%s) && systemctl restart turtlebot4.service'"
  rc=$?
  unset pw
  ssh -S "$sock" -O exit "$user@$ip" 2>/dev/null
  if [ $rc -eq 0 ]; then
    echo "[ok]   reloj del robot igualado y bringup reiniciado. Espera ~30 s y relanza SLAM/Nav2 si estaban corriendo."
  else
    echo "[FALLA] no se pudo fijar la hora (rc=$rc)."
  fi
  return $rc
}

_tb4_auto_possible() {
  # true si hay clave SSH y sudo sin contrasena para fijar la hora (ver configurar_sync_automatico.sh)
  local ip="$1" user="${TB4_USER:-ubuntu}"
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$user@$ip" \
    'sudo -n -l "$(command -v date)" -u -s @1 >/dev/null 2>&1' >/dev/null 2>&1
}

_tb4_sync_auto() {
  # Fija la hora del robot sin contrasenas. Si el desfase pasa de 300 s reinicia el bringup y espera a que vuelva.
  local ip="$1" off="$2" user="${TB4_USER:-ubuntu}" restart=0 cmd i n=0
  awk -v o="$off" 'BEGIN{exit !(o>300 || o<-300)}' && restart=1
  echo "[..]   sincronizando el reloj del robot automaticamente..."
  cmd="sudo -n \"\$(command -v date)\" -u -s @$(date +%s)"
  [ "$restart" -eq 1 ] && cmd="$cmd && sudo -n \"\$(command -v systemctl)\" restart turtlebot4.service"
  cmd="$cmd; curl -s -X POST http://192.168.186.2/api/restart-ntpd >/dev/null 2>&1 || true"
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$user@$ip" "$cmd" >/dev/null || { echo "[FALLA] no pude fijar la hora"; return 1; }
  if [ "$restart" -eq 1 ]; then
    echo "[..]   bringup reiniciado; esperando a que vuelva (max ~90 s)..."
    for i in $(seq 1 15); do
      sleep 5; n=$(_tb4_key_topics)
      [ "$n" -ge 3 ] && break
    done
    [ "$n" -ge 3 ] || echo "[AVISO] el robot tarda en volver: repite 'source tb4_connect.sh' en un minuto."
  fi
  echo "[ok]   reloj del robot igualado. El Create 3 tarda unos minutos en alinearse: NO lances SLAM hasta que  python3 scripts/chequeo_reloj_tb4.py --esperar  diga OK."
}

_tb4_check_clock() {
  local ip="$1" off
  off=$(_tb4_clock_offset)
  [ "$off" = "none" ] || [ -z "$off" ] && { echo "[..]   reloj: no pude medir (sin /odom)"; return 0; }
  if awk -v o="$off" 'BEGIN{exit !(o>2 || o<-2)}'; then
    printf '[AVISO] reloj del robot desfasado %.0f s (~%.1f dias) respecto a esta laptop.\n' \
      "$off" "$(awk -v o="$off" 'BEGIN{print o/86400}')"
    # Solo se corrige un desfase enorme (> 1 h, p. ej. el robot arranco en 2025). Un salto pequeno en la Pi
    # deja al Create 3 atrasado unos minutos y SLAM descarta todos los escaneos: peor que no tocar nada.
    if awk -v o="$off" 'BEGIN{exit !(o<3600 && o>-3600)}'; then
      echo "        Desfase pequeno (<1 h): no se corrige a proposito. Lo que importa es que Pi y Create 3 coincidan:"
      echo "        python3 scripts/chequeo_reloj_tb4.py   (forzar igual: source tb4_connect.sh --hora)"
      return 0
    fi
    if _tb4_auto_possible "$ip"; then _tb4_sync_auto "$ip" "$off"; return $?; fi
    echo "        Sincronizar reinicia turtlebot4.service del robot. (Para hacerlo solo: ./scripts/configurar_sync_automatico.sh)"
    [ -t 0 ] || { echo "        (terminal no interactiva) Hazlo luego: source tb4_connect.sh --hora"; return 0; }
    local ans
    read -r -p "        Sincronizar ahora? [S/n] " ans
    case "$ans" in n|N) echo "        Omitido. Luego: source tb4_connect.sh --hora" ;; *) _tb4_sync_clock "$ip" ;; esac
  else
    echo "[ok]   reloj robot-laptop sincronizado (${off}s)"
  fi
}

_tb4_connect() {
  local cfg="$HOME/.tb4_robot"

  if [ "$1" = "--off" ]; then
    unset ROS_STATIC_PEERS TB4_IP TB4_MODE
    export ROS_DOMAIN_ID=0
    ros2 daemon stop >/dev/null 2>&1
    echo "Configuracion TB4 limpiada (ROS_DOMAIN_ID=0, sin peers)."
    return 0
  fi

  if [ "$1" = "--hora" ]; then
    # Solo sincronizar el reloj con el robot guardado (~/.tb4_robot).
    [ -f "$cfg" ] || { echo "No hay robot guardado: conecta primero con source tb4_connect.sh <IP> <DOMAIN_ID>"; return 1; }
    # shellcheck disable=SC1090
    source "$cfg"
    [ "$ROS_DISTRO" = "jazzy" ] || source /opt/ros/jazzy/setup.bash || return 1
    _tb4_sync_clock "$TB4_IP"
    return $?
  fi

  local ip="$1" domain="$2" mode="${3:-auto}"
  if [ -z "$ip" ]; then
    if [ ! -f "$cfg" ]; then
      echo "No hay robot guardado. Uso: source tb4_connect.sh <IP_ROBOT> <DOMAIN_ID>"
      return 1
    fi
    # shellcheck disable=SC1090
    source "$cfg"
    ip="$TB4_IP"; domain="$ROS_DOMAIN_ID"; mode="${TB4_MODE:-auto}"
  fi

  if ! [[ "$domain" =~ ^[0-9]+$ ]] || [ "$domain" -gt 232 ]; then
    echo "DOMAIN_ID invalido: '$domain' (usa 0-101)."
    return 1
  fi
  [ "$domain" -gt 101 ] && echo "Aviso: ROS_DOMAIN_ID=$domain > 101 puede chocar con puertos de Linux."
  case "$mode" in auto|multicast|peers) ;; *) echo "Modo invalido: $mode"; return 1 ;; esac

  [ "$ROS_DISTRO" = "jazzy" ] || source /opt/ros/jazzy/setup.bash || return 1
  export ROS_DOMAIN_ID="$domain"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export TB4_IP="$ip"
  unset ROS_STATIC_PEERS

  echo "== TurtleBot 4 en $ip | ROS_DOMAIN_ID=$domain | modo $mode"

  if ping -c 2 -W 2 "$ip" >/dev/null 2>&1; then
    echo "[ok]   ping a $ip"
  else
    echo "[FALLA] ping a $ip: no hay ruta IP. Misma red? IP correcta? (TROUBLESHOOTING 1.1 y 1.4)"
    return 1
  fi

  local found=0 used=""
  ros2 daemon stop >/dev/null 2>&1
  if [ "$mode" != "multicast" ]; then
    # Preferir unicast (ROS_STATIC_PEERS): en Wi-Fi el multicast se envia a tasa minima y satura la camara.
    export ROS_STATIC_PEERS="$ip"
    found=$(_tb4_key_topics); used="peers"
    echo "[..]   descubrimiento con ROS_STATIC_PEERS=$ip: $found/4 topicos clave"
  fi
  if [ "$found" -lt 4 ] && [ "$mode" != "peers" ]; then
    unset ROS_STATIC_PEERS
    local found_mc
    found_mc=$(_tb4_key_topics)
    echo "[..]   descubrimiento multicast: $found_mc/4 topicos clave"
    if [ "$found_mc" -gt "$found" ]; then
      found=$found_mc; used="multicast"
    elif [ "$used" = "peers" ]; then
      export ROS_STATIC_PEERS="$ip"
    fi
  fi
  [ "$used" = "multicast" ] && unset ROS_STATIC_PEERS
  export TB4_MODE="$used"

  if [ "$found" -eq 0 ]; then
    echo "[FALLA] ping OK pero DDS no ve el robot. Revisa ROS_DOMAIN_ID en el robot,"
    echo "        que el bringup este activo, y TROUBLESHOOTING 1.1. Diagnostico: ./tb4_check.sh --robot"
    return 1
  fi

  {
    echo "# Generado por tb4_connect.sh el $(date '+%F %T')"
    echo "export TB4_IP=$ip"
    echo "export TB4_MODE=$used"
    echo "export ROS_DOMAIN_ID=$domain"
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    [ "$used" = "peers" ] && echo "export ROS_STATIC_PEERS=$ip"
  } > "$cfg"

  if [ "$found" -eq 4 ]; then
    echo "[ok]   conectado ($used). Guardado en $cfg"
  else
    echo "[AVISO] conectado ($used) pero faltan topicos clave. Guardado en $cfg. Ejecuta: ./tb4_check.sh"
  fi

  # Reloj: desactivar con TB4_NOSYNC=1 antes de hacer source.
  [ -n "$TB4_NOSYNC" ] || _tb4_check_clock "$ip"
}

_tb4_connect "$@"
