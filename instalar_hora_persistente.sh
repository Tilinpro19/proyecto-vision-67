#!/usr/bin/env bash
# Corre DENTRO DEL ROBOT (Raspberry Pi) con sudo. Hace que la Pi recuerde la hora entre apagados:
# sin reloj con pila ni internet arranca siempre en 2025. Este servicio guarda la hora cada 5 min y al
# apagar, y la restaura al arrancar (antes de chrony y del bringup). El Create 3 copia luego la hora de la Pi.
#
# Uso:
#   sudo ./instalar_hora_persistente.sh            # instala y activa
#   sudo ./instalar_hora_persistente.sh --quitar   # desinstala
#   ./instalar_hora_persistente.sh --estado        # muestra el estado (sin sudo)
#
# Queda la hora del ULTIMO guardado (max ~5 min antes del apagado): atrasada lo que dure el apagon,
# pero coherente. Para igualarla con la laptop sigue valiendo: source tb4_connect.sh --hora
# Crea: /usr/local/sbin/hora-persistente, /etc/systemd/system/hora-persistente.{service,timer},
#       /var/lib/hora_persistente (la hora guardada).

BIN=/usr/local/sbin/hora-persistente
SVC=/etc/systemd/system/hora-persistente.service
TMR=/etc/systemd/system/hora-persistente.timer

if [ "$1" = "--estado" ]; then
  echo "hora actual:   $(date -u '+%F %T') UTC"
  [ -f /var/lib/hora_persistente ] && echo "hora guardada: $(date -u -d @"$(cat /var/lib/hora_persistente)" '+%F %T') UTC" \
    || echo "hora guardada: (aun no hay)"
  systemctl is-enabled hora-persistente.service hora-persistente.timer 2>&1
  systemctl is-active hora-persistente.timer 2>&1
  exit 0
fi

if [ "$(id -u)" -ne 0 ]; then echo "Ejecuta con sudo: sudo $0 $*"; exit 1; fi

if [ "$1" = "--quitar" ]; then
  systemctl disable --now hora-persistente.timer hora-persistente.service 2>/dev/null
  rm -f "$BIN" "$SVC" "$TMR" /var/lib/hora_persistente
  systemctl daemon-reload
  echo "Desinstalado."
  exit 0
fi

cat > "$BIN" <<'EOF'
#!/usr/bin/env bash
# hora-persistente save|restore  (HORA_FILE y DRY=1 solo para pruebas)
FILE="${HORA_FILE:-/var/lib/hora_persistente}"
case "$1" in
  save)
    now=$(date +%s)
    # no guardar una hora absurda (el reloj aun en 2025 tras un arranque sin restaurar)
    saved=$(cat "$FILE" 2>/dev/null); saved=${saved:-0}
    [ "$now" -gt "$saved" ] && echo "$now" > "$FILE.tmp" && mv "$FILE.tmp" "$FILE"
    ;;
  restore)
    saved=$(cat "$FILE" 2>/dev/null)
    now=$(date +%s)
    if [[ "$saved" =~ ^[0-9]+$ ]] && [ "$saved" -gt 1700000000 ] && [ "$saved" -gt "$now" ]; then
      if [ -n "$DRY" ]; then echo "[dry] pondria la hora en @$saved"; else date -u -s "@$saved" >/dev/null && echo "hora restaurada a $(date -u '+%F %T') UTC"; fi
    else
      echo "nada que restaurar (guardada=${saved:-ninguna}, actual=$now)"
    fi
    ;;
  *) echo "uso: $0 save|restore"; exit 1 ;;
esac
EOF
chmod 755 "$BIN"

cat > "$SVC" <<EOF
[Unit]
Description=Guardar/restaurar la hora (Pi sin RTC)
After=local-fs.target
Before=chrony.service turtlebot4.service
DefaultDependencies=no

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=$BIN restore
ExecStop=$BIN save

[Install]
WantedBy=multi-user.target
EOF

cat > "$TMR" <<EOF
[Unit]
Description=Guardar la hora cada 5 min

[Timer]
OnBootSec=5min
OnUnitActiveSec=5min

[Install]
WantedBy=timers.target
EOF

# El timer llama al mismo binario; servicio aparte para el guardado periodico
cat > /etc/systemd/system/hora-persistente-guardar.service <<EOF
[Unit]
Description=Guardar la hora ahora

[Service]
Type=oneshot
ExecStart=$BIN save
EOF
sed -i 's/^\[Timer\]/[Timer]\nUnit=hora-persistente-guardar.service/' "$TMR"

systemctl daemon-reload
"$BIN" save
systemctl enable --now hora-persistente.service hora-persistente.timer
echo
echo "Instalado. Hora guardada ahora: $(date -u -d @"$(cat /var/lib/hora_persistente)" '+%F %T') UTC"
echo "Comprueba el ciclo completo: sincroniza la hora, espera >5 min (o: sudo hora-persistente save), reinicia la Pi y mira la fecha."
echo "Para quitarlo: sudo $0 --quitar"
