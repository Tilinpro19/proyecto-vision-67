#!/usr/bin/env bash
# ==============================================================================
# PARADA DE EMERGENCIA INSTANTANEA PARA TURTLEBOT 4
# ==============================================================================
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$DIR/proyecto_final/1_competencia/parar.sh" "$@"
