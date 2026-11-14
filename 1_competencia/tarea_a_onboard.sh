#!/usr/bin/env bash
# ==============================================================================
# Tarea A On-Board: ejecuta la mision A -> B -> A directamente en la Raspberry Pi 4.
# El control NO depende de la red: la laptop solo lanza esto por SSH (cable o Wi-Fi) y
# lee la terminal / el JSON de resultado.
#
#   ssh ubuntu@<ip-robot>
#   cd ~/turtleclaude4/1_competencia && ./tarea_a_onboard.sh --goal 3.0 0.5 --param cuenta=5
#
# Primera vez, con el robot quieto (NO publica /cmd_vel): comprueba sensores y mide el CPU
# del calculo; el robot no se mueve, asi que la pose no avanza. Mira "perfil" en el JSON.
#   ./tarea_a_onboard.sh --goal 3.0 0.5 --dry
# ==============================================================================
cd "$(dirname "$0")" || exit 1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=67

# La Pi tiene 4 nucleos que comparten el bringup, el RPLIDAR y esta mision. NumPy/OpenBLAS
# abre un hilo por nucleo incluso para matrices diminutas: se gasta CPU sin ganar velocidad.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

# Sincronizar reloj de Create 3 preventivamente
curl -s -X POST http://192.168.186.2/api/restart-ntpd > /dev/null 2>&1 || true

mkdir -p resultados
RES="resultados/tarea_a_$(date +%Y%m%d_%H%M%S).json"
echo "[ON-BOARD] Tarea A en la Raspberry Pi (resultado: $RES)"
# --voz ninguna: el nombre sale por terminal, en el JSON y por /nombre_reconocido + pitido.
exec python3 mision.py --tarea A --voz ninguna --resultado "$RES" "$@"
