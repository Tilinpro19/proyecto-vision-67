#!/usr/bin/env bash
# Corre en la LAPTOP (WSL), UNA sola vez. Deja al robot listo para que tb4_connect.sh sincronice la hora
# solo, sin contrasenas:
#   1) clave SSH de esta laptop en el robot (te pide la contrasena del robot una vez)
#   2) permiso de sudo SIN clave solo para: fijar la hora (date -u -s ...) y reiniciar turtlebot4.service
#   3) instala la hora persistente en la Pi (instalar_hora_persistente.sh): no vuelve a 2025 tras un apagon
#
# Uso:  ./configurar_sync_automatico.sh [IP_ROBOT]      (por defecto la guardada en ~/.tb4_robot)
#       ./configurar_sync_automatico.sh --quitar [IP]   (quita el permiso de sudo y la hora persistente)
# Usuario del robot: ubuntu (cambia con TB4_USER). Pide la contrasena de sudo del robot una vez.

QUITAR=0
[ "$1" = "--quitar" ] && { QUITAR=1; shift; }
USER_R="${TB4_USER:-ubuntu}"
IP="$1"
if [ -z "$IP" ] && [ -f "$HOME/.tb4_robot" ]; then IP=$(grep -E '^export TB4_IP=' "$HOME/.tb4_robot" | cut -d= -f2); fi
[ -n "$IP" ] || { echo "Falta la IP: $0 <IP_ROBOT>"; exit 1; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE=/tmp/tb4_setup_remoto.sh

# Script que corre en el robot, con sudo
mkdir -p "$HOME/.cache" && TMP=$(mktemp)
cat > "$TMP" <<EOF
#!/usr/bin/env bash
U="$USER_R"
QUITAR=$QUITAR
H=\$(getent passwd "\$U" | cut -d: -f6)   # sudo cambia \$HOME a /root: usar el home del usuario
if [ "\$QUITAR" = "1" ]; then
  rm -f /etc/sudoers.d/tb4-hora
  [ -f "\$H/instalar_hora_persistente.sh" ] && bash "\$H/instalar_hora_persistente.sh" --quitar
  echo "Quitado: permiso de sudo para la hora y servicio de hora persistente."
  exit 0
fi
DATE_BIN=\$(command -v date); SYSTEMCTL_BIN=\$(command -v systemctl)
T=\$(mktemp)
echo "\$U ALL=(root) NOPASSWD: \$DATE_BIN -u -s *, \$SYSTEMCTL_BIN restart turtlebot4.service" > "\$T"
if visudo -cf "\$T" >/dev/null; then
  install -m 440 -o root -g root "\$T" /etc/sudoers.d/tb4-hora
  echo "Permiso de sudo instalado (/etc/sudoers.d/tb4-hora): solo date -u -s y reiniciar turtlebot4.service."
else
  echo "ERROR: la regla de sudoers no valida; no se instalo nada."; rm -f "\$T"; exit 1
fi
rm -f "\$T"
if [ -f "\$H/instalar_hora_persistente.sh" ]; then bash "\$H/instalar_hora_persistente.sh"
else echo "(sin instalar_hora_persistente.sh en el robot: hora persistente omitida)"; fi
EOF

if [ "$QUITAR" = "0" ]; then
  [ -f ~/.ssh/id_ed25519 ] || { echo "== Creando clave SSH"; ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519 -q; }
  echo "== 1/3 Copiando la clave SSH al robot (contrasena del robot)"
  ssh-copy-id -i ~/.ssh/id_ed25519.pub "$USER_R@$IP" || { echo "FALLO ssh-copy-id"; exit 1; }
  [ -f "$HERE/instalar_hora_persistente.sh" ] && scp -q "$HERE/instalar_hora_persistente.sh" "$USER_R@$IP:~/" && \
    ssh "$USER_R@$IP" 'chmod +x ~/instalar_hora_persistente.sh'
fi
echo "== 2/3 y 3/3 Permiso de sudo y hora persistente en el robot (contrasena de sudo)"
scp -q "$TMP" "$USER_R@$IP:$REMOTE" && ssh -t "$USER_R@$IP" "sudo bash $REMOTE; rm -f $REMOTE"
rm -f "$TMP"

if [ "$QUITAR" = "0" ]; then
  echo "== Comprobando acceso automatico (sin contrasenas)"
  if ssh -o BatchMode=yes "$USER_R@$IP" 'sudo -n -l "$(command -v date)" -u -s @1 >/dev/null 2>&1'; then
    echo "[ok] Listo: tb4_connect.sh sincronizara la hora solo. (Quita TB4_NOSYNC de ~/.bashrc si lo pusiste.)"
  else
    echo "[FALLA] no funciona el acceso sin contrasena; tb4_connect.sh seguira preguntando."
  fi
fi
