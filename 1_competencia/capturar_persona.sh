#!/usr/bin/env bash
# ==============================================================================
# CAPTURA RAPIDA DE PERSONA PARA LA GALERIA (DIA DE LA COMPETENCIA)
# Uso: ./capturar_persona.sh Nombre_Apellido [num_fotos]
# ==============================================================================
set -e

NOMBRE="${1:-}"
NUM="${2:-10}"

if [ -z "$NOMBRE" ]; then
    echo "Uso: $0 <Nombre_Apellido> [numero_de_fotos]"
    echo "Ejemplo: $0 Juan_Perez 10"
    exit 1
fi

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "=========================================================="
echo " CAPTURANDO $NUM FOTOS PARA: $NOMBRE"
echo " Apunta la camara del robot hacia la foto de la persona."
echo "=========================================================="

python3 facial.py --robot --capturar "$NOMBRE" --n "$NUM"

echo ""
echo "=========================================================="
echo " PROBANDO RECONOCIMIENTO CON LA GALERIA ACTUALIZADA..."
echo "=========================================================="

python3 facial.py --probar

echo ""
echo "[OK] Persona '$NOMBRE' registrada y verificada exitosamente en la galeria."
