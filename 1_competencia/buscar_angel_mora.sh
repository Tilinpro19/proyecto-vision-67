#!/usr/bin/env bash
# Busca a angel_mora en el circuito guiado por senales, le toma foto, regresa a A y anuncia.
cd "$(dirname "$0")" || exit 1
exec ./tarea_a.sh --modo guiado --persona angel_mora --camara-hd --ver "$@"
