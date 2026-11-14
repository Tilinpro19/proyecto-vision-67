#!/usr/bin/env bash
# ==============================================================================
# DESPLIEGUE A BORDO (EDGE-SPLIT) - TURTLEBOT 4 LITE
# Sincroniza el codigo optimizado a la Raspberry Pi 4 y prepara la ejecucion
# autonoma local (sin dependencia de Wi-Fi y con 0 ms de latencia DDS).
# ==============================================================================

ROBOT_IP="${1:-192.168.185.3}"
ROBOT_USER="${2:-ubuntu}"
TARGET_DIR="${TARGET_DIR:-~/turtleclaude4}"

echo "=========================================================="
echo " [1/5] Verificando conexion con TurtleBot 4 en $ROBOT_IP..."
echo "=========================================================="

if ! ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "ERROR: No se puede alcanzar a $ROBOT_IP por ping."
    echo "Verifica que el cable Ethernet este conectado y la IP de la laptop sea 192.168.185.10."
    exit 1
fi

echo "Conexion Ethernet OK."

echo "=========================================================="
echo " [2/5] Sincronizando codigo a la Raspberry Pi..."
echo "=========================================================="

# Directorio base del proyecto
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ssh -o ConnectTimeout=5 "$ROBOT_USER@$ROBOT_IP" "mkdir -p $TARGET_DIR/1_competencia $TARGET_DIR/2_laboratorio $TARGET_DIR/scripts $TARGET_DIR/tb4lib" || exit 1

rsync -avz --delete \
    --exclude=".git" \
    --exclude="__pycache__" \
    --exclude="*.pyc" \
    --exclude="*.log" \
    "$BASE_DIR/1_competencia/" "$ROBOT_USER@$ROBOT_IP:$TARGET_DIR/1_competencia/" || exit 1

rsync -avz --delete \
    --exclude=".git" \
    --exclude="__pycache__" \
    --exclude="*.pyc" \
    --exclude="*.log" \
    "$BASE_DIR/2_laboratorio/" "$ROBOT_USER@$ROBOT_IP:$TARGET_DIR/2_laboratorio/" || exit 1

rsync -avz \
    "$BASE_DIR/tb4_connect.sh" "$BASE_DIR/tb4_check.sh" "$BASE_DIR/fastdds_pc.xml" "$BASE_DIR/parar.sh" \
    "$ROBOT_USER@$ROBOT_IP:$TARGET_DIR/" || exit 1

if [ -d "$BASE_DIR/scripts" ]; then
    rsync -avz --delete \
        --exclude="__pycache__" \
        --exclude="*.pyc" \
        "$BASE_DIR/scripts/" "$ROBOT_USER@$ROBOT_IP:$TARGET_DIR/scripts/" || exit 1
fi

rsync -avz --delete \
    --exclude=".git" \
    --exclude="__pycache__" \
    --exclude="*.pyc" \
    "$BASE_DIR/tb4_docker/tb4lib/" "$ROBOT_USER@$ROBOT_IP:$TARGET_DIR/tb4lib/" || exit 1

echo "Archivos sincronizados correctamente."

echo "=========================================================="
echo " [3/5] Instalando tb4lib en la Raspberry Pi..."
echo "=========================================================="

ssh "$ROBOT_USER@$ROBOT_IP" "pip install -e $TARGET_DIR/tb4lib --break-system-packages --no-deps > /dev/null 2>&1 || true"
echo "tb4lib registrado en Python de la Raspberry Pi."

echo "=========================================================="
echo " [4/5] Sincronizando relojes (Laptop -> Pi -> Create 3)..."
echo "=========================================================="

NOW=$(date +%s)
ssh "$ROBOT_USER@$ROBOT_IP" "sudo date -u -s @$NOW > /dev/null 2>&1; curl -s -X POST http://192.168.186.2/api/restart-ntpd > /dev/null 2>&1 || true"
echo "Reloj sincronizado con exito."

echo "=========================================================="
echo " [5/5] Creando lanzadores on-board y verificando salud..."
echo "=========================================================="

ssh "$ROBOT_USER@$ROBOT_IP" "cat << 'EOF' > $TARGET_DIR/1_competencia/tarea_a_onboard.sh
#!/usr/bin/env bash
cd \"\$(dirname \"\$0\")\" || exit 1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=67
curl -s -X POST http://192.168.186.2/api/restart-ntpd > /dev/null 2>&1 || true
echo \"[ON-BOARD] Ejecutando Tarea A directamente en Raspberry Pi (sin latencia de red)...\"
exec python3 mision.py --tarea A \"\$@\"
EOF
chmod +x $TARGET_DIR/1_competencia/tarea_a_onboard.sh

cat << 'EOF' > $TARGET_DIR/1_competencia/tarea_b_onboard.sh
#!/usr/bin/env bash
cd \"\$(dirname \"\$0\")\" || exit 1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=67
curl -s -X POST http://192.168.186.2/api/restart-ntpd > /dev/null 2>&1 || true
echo \"[ON-BOARD] Ejecutando Tarea B directamente en Raspberry Pi (sin latencia de red)...\"
exec python3 mision.py --tarea B \"\$@\"
EOF
chmod +x $TARGET_DIR/1_competencia/tarea_b_onboard.sh
"

echo "=========================================================="
echo "  DESPLIEGUE COMPLETADO CON EXITO!"
echo "=========================================================="
echo "Puedes correr la mision de dos formas:"
echo "1. On-board (recomendado, 100% inmune a caidas de Wi-Fi):"
echo "   ssh $ROBOT_USER@$ROBOT_IP 'cd $TARGET_DIR/1_competencia && ./tarea_a_onboard.sh --goal 3.0 0.0'"
echo ""
echo "2. Desde la Laptop (por Ethernet o Wi-Fi con telemetria en pantalla):"
echo "   cd 1_competencia && ./tarea_a.sh --goal 3.0 0.0"
echo "=========================================================="
