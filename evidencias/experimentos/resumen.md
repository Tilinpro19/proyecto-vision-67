# Experimentación en el robot real — 2026-11-23 21:00

Generado por `scripts/recopilar_experimentos.py resumen`. Datos **medidos en el robot** (los del banco simulado van aparte, en `sim/`).

## Tarea A

| modo | config | intentos | completos | t total (mín / mediana) | choques | reconocimiento seguro |
|---|---|---|---|---|---|---|
| — | config.json | 9 | 3 | 51.2 / 51.3 s | 2 | 0/9 |
| auto | config_agresivo.json | 1 | 1 | 69.0 / 69.0 s | 0 | 0/1 |
| auto | config_rapido.json | 1 | 1 | 105.3 / 105.3 s | 0 | 1/1 |
| guiado | config.json | 9 | 0 | — | 3 | 0/9 |

| fecha | config | ok | t total | ida | escaneo B | regreso | choques | decisiones | callejones | persona → reconocida (score) | QR | ancho | oficial | observaciones |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-09 22:32:15 | config.json | no (PARADA: Ctrl+C) | 10.20 | — | — | — | 1 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-09 22:33:27 | config.json | no (PARADA: Ctrl+C) | 15.17 | — | — | — | 1 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-09 22:33:49 | config.json | no (PARADA: Ctrl+C) | 0.00 | — | — | — | 0 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-09 22:44:29 | config.json | no (PARADA: Ctrl+C) | 43.55 | — | — | — | 0 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-09 22:45:20 | config.json | no (PARADA: Ctrl+C) | 41.81 | — | — | — | 0 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-09 22:48:20 | config.json | no (PARADA: Ctrl+C) | 41.61 | — | — | — | 0 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-11 18:01:49 | config.json | sí | 51.34 | 17.49 | 15.04 | 18.49 | 0 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-11 18:14:02 | config.json | sí | 51.31 | 17.55 | 15.06 | 18.38 | 0 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-11 18:23:13 | config.json | sí | 51.24 | 17.59 | 15.02 | 18.31 | 0 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 16:04:21 | config.json | no (SEGUIR) | 8.83 | — | — | — | 1 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 16:07:34 | config.json | no (PARADA: supero t_max = 10 s) | 10.02 | — | — | — | 0 | 0 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 16:35:41 | config.json | no (PARADA: Ctrl+C) | 91.32 | — | — | — | 0 | 8 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 16:42:59 | config.json | no (PARADA: Ctrl+C) | 59.65 | — | — | — | 0 | 2 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 16:49:12 | config.json | no (PARADA: Ctrl+C) | 89.77 | — | — | — | 1 | 8 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 17:35:55 | config.json | no (PARADA: supero t_max = 300 s) | 300.17 | — | — | — | 1 | 27 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 18:11:36 | config.json | no (PARADA: Ctrl+C) | 74.85 | — | — | — | 0 | 3 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 18:14:35 | config.json | no (PARADA: supero t_max = 90 s) | 90.23 | — | — | — | 0 | 1 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-13 18:25:03 | config.json | no (PARADA: supero t_max = 90 s) | 90.01 | — | — | — | 0 | 13 | 0 | — → — (0.00, dudoso) | — | — | — | — |
| 2026-11-23 20:45:55 | config_rapido.json | sí | 105.32 | 46.49 | 31.34 | 27.17 | 0 | 6 | 0 | Angel_Mora → Angel_Mora (0.58) | milestone_beta | — | — | — |
| 2026-11-23 20:48:24 | config_agresivo.json | sí | 69.05 | 30.10 | 17.54 | 21.09 | 0 | 4 | 0 | Angel_Mora → Alvaro_Garcia (0.00, dudoso) | — | — | — | — |

Fotos del reconocimiento:

- 2026-11-23 20:45:55: Angel_Mora (0.58) — `evidencias/experimentos/robot/mision_directo/logs/mision_A_20261123_204401_reconocido_Angel_Mora.jpg`

## Último estado del robot

fecha_pc: 2026-11-23 21:00:57, bateria: 84 %, en_dock: True, temp_c: 54, carga: 0.99 0.95 1.23, desfase_reloj_s: -3.3

