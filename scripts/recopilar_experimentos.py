#!/usr/bin/env python3
"""
Recopila la experimentacion del TurtleBot 4 para el informe: intentos REALES (lo que deja
mision.py en el robot) y el banco SIMULADO, en una carpeta y una tabla.

  python3 scripts/recopilar_experimentos.py traer            # baja intentos nuevos de la Pi (por cable)
  python3 scripts/recopilar_experimentos.py estado           # bateria, dock, reloj, temperatura, CPU
  python3 scripts/recopilar_experimentos.py resumen          # intentos.csv + resumen.md
  python3 scripts/recopilar_experimentos.py sim              # banco simulado de los 3 perfiles (~2 min)
  python3 scripts/recopilar_experimentos.py todo             # traer + estado + resumen

Opciones: --ip 192.168.185.3  --carpeta mision_directo (en el home del robot; repetible, p. ej. mision_mov)
          --salida evidencias/experimentos

Todo queda en evidencias/experimentos/:
  robot/<carpeta>/resultados/*.json, robot/<carpeta>/logs/*.log|*.npz|*.jpg   (copias, nunca se borra nada)
  robot/estado_<fecha>.json                    (lo que devuelve 'estado')
  sim/bench_<fecha>.csv                        (SIMULADO: nunca mezclar con los tiempos reales)
  notas.csv                                    (A MANO: ancho de pasillo, intento oficial, observaciones)
  intentos.csv, resumen.md                     (lo que arma 'resumen')

Tambien lee los intentos corridos en esta maquina (1_competencia/resultados y 1_competencia/logs).
Solo lee del robot: no mueve nada ni cambia su configuracion.
"""
import argparse
import csv
import glob
import json
import os
import re
import statistics
import subprocess
import sys
import time
from datetime import datetime

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMP = os.path.join(REPO, "1_competencia")
SALIDA = os.path.join(REPO, "evidencias", "experimentos")
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]
NOTAS_COLS = ["archivo", "ancho_pasillo_m", "oficial", "observaciones"]
LINEA_LOG = re.compile(r"^\[\s*([\d.]+)s\]\s+(\S+)\s+(.*)$")
TRANS = re.compile(r"^(\S+) -> (\S+)(?: \((.*)\))?$")


# --- traer ---------------------------------------------------------------------------------
def traer(ip, carpetas, salida, user="ubuntu"):
    for carpeta in carpetas:
        # relativa al home del robot ('~/x', 'x' o una ruta que la shell local ya expandio)
        nombre = os.path.basename(carpeta.replace("\\", "/").rstrip("/")) or "robot"
        carpeta = nombre
        dest = os.path.join(salida, "robot", nombre)
        lista = subprocess.run(SSH + [f"{user}@{ip}", f"cd {carpeta} 2>/dev/null && find resultados logs -type f 2>/dev/null"],
                               capture_output=True, text=True, timeout=30)
        if lista.returncode not in (0, 1):
            print(f"[FALLA] no pude listar {carpeta} en {ip}: {lista.stderr.strip()}")
            continue
        remotos = [l for l in lista.stdout.splitlines() if l.strip()]
        nuevos = [r for r in remotos if not os.path.isfile(os.path.join(dest, r))]
        print(f"{carpeta}: {len(remotos)} archivos en el robot, {len(nuevos)} nuevos")
        if not nuevos:
            continue
        os.makedirs(dest, exist_ok=True)
        tar = subprocess.run(SSH + [f"{user}@{ip}", f"cd {carpeta} && tar -cf - -T -"],
                             input=("\n".join(nuevos) + "\n").encode(), capture_output=True, timeout=600)
        if tar.returncode != 0:
            print(f"[FALLA] tar en el robot: {tar.stderr.decode(errors='replace').strip()}")
            continue
        sub = subprocess.run(["tar", "-xf", "-", "-C", dest], input=tar.stdout, capture_output=True)
        if sub.returncode != 0:
            print(f"[FALLA] tar local: {sub.stderr.decode(errors='replace').strip()}")
            continue
        print(f"  -> {dest}")


# --- estado --------------------------------------------------------------------------------
ESTADO_REMOTO = r"""
echo "fecha_pi=$(date -u +%s)"
echo "uptime=$(uptime -p)"
echo "carga=$(cut -d' ' -f1-3 /proc/loadavg)"
echo "temp_c=$(( $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0) / 1000 ))"
echo "disco=$(df -h / | awk 'NR==2{print $5}')"
echo "bringup=$(systemctl is-active turtlebot4 2>/dev/null)"
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; source /etc/turtlebot4/setup.bash >/dev/null 2>&1
echo "topicos=$(timeout 15 ros2 topic list </dev/null 2>/dev/null | wc -l)"
echo "bateria=$(timeout 15 ros2 topic echo --once /battery_state --field percentage </dev/null 2>/dev/null | grep -E '^[0-9.]+$' | head -1)"
echo "en_dock=$(timeout 15 ros2 topic echo --once /dock_status --field is_docked </dev/null 2>/dev/null | grep -iE '^(true|false)$' | head -1)"
echo "scan_activo=$(timeout 15 ros2 topic echo --once /scan --field header.stamp.sec </dev/null 2>/dev/null | grep -cE '^[0-9]+$')"
"""


def estado(ip, salida, user="ubuntu"):
    t_pc = time.time()
    # en bytes: con text=True, Python en Windows manda CRLF y 'source setup.bash<CR>' falla en
    # silencio (queda el dominio ROS 0: solo 2 topicos)
    r = subprocess.run(SSH + [f"{user}@{ip}", "bash -s"], input=ESTADO_REMOTO.encode(), capture_output=True,
                       timeout=90)
    out = r.stdout.decode(errors="replace")
    if r.returncode != 0 and not out:
        print(f"[FALLA] no pude leer el estado de {ip}: {r.stderr.decode(errors='replace').strip()}")
        return None
    est = dict(l.split("=", 1) for l in out.splitlines() if "=" in l)
    est["fecha_pc"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        est["desfase_reloj_s"] = round(float(est.get("fecha_pi", "nan")) - t_pc, 1)
    except ValueError:
        pass
    try:
        est["bateria"] = f"{float(est['bateria']) * 100:.0f} %"
    except (KeyError, ValueError):
        pass
    os.makedirs(os.path.join(salida, "robot"), exist_ok=True)
    ruta = os.path.join(salida, "robot", f"estado_{time.strftime('%Y%m%d_%H%M%S')}.json")
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(est, f, indent=2, ensure_ascii=False)
    for k in ("bateria", "en_dock", "scan_activo", "bringup", "topicos", "temp_c", "carga", "desfase_reloj_s", "uptime", "disco"):
        print(f"  {k:16s} {est.get(k, '?')}")
    if abs(est.get("desfase_reloj_s", 0)) > 5:
        print("  [AVISO] reloj de la Pi desfasado: source tb4_connect.sh --hora")
    print(f"  -> {ruta}")
    return est


# --- resumen -------------------------------------------------------------------------------
def _leer_log(ruta):
    """(transiciones, eventos) de un .log de mision.py."""
    trans, eventos = [], []
    try:
        with open(ruta, encoding="utf-8", errors="replace") as f:
            for linea in f:
                m = LINEA_LOG.match(linea.rstrip("\n"))
                if not m:
                    continue
                t, texto = float(m.group(1)), m.group(3)
                eventos.append(linea.rstrip("\n"))
                tm = TRANS.match(texto)
                if tm:
                    trans.append(dict(t=t, estado=tm.group(2), motivo=tm.group(3) or ""))
    except OSError:
        pass
    return trans, eventos


def _stamp(nombre):
    m = re.search(r"(\d{8}_\d{6})", nombre)
    return datetime.strptime(m.group(1), "%Y%m%d_%H%M%S") if m else None


def _log_cercano(json_path, logs):
    """Log de la misma corrida para JSONs viejos (sin 'log'): el de fecha mas cercana, anterior."""
    t = _stamp(os.path.basename(json_path))
    if t is None:
        return None
    # el JSON lleva la hora del lanzador (antes) y el log la de mision.py (segundos despues)
    cands = [(t - _stamp(os.path.basename(l))).total_seconds() for l in logs]
    pares = [(abs(d), l) for d, l in zip(cands, logs) if -120 <= d < 1200]
    return min(pares)[1] if pares else None


def _t(trans, *estados):
    return next((x["t"] for x in trans if x["estado"] in estados), None)


def _res_de_log(ruta):
    """Intento que solo dejo el .log (sin JSON): el dict de su linea 'RESULTADO: {...}'."""
    with open(ruta, encoding="utf-8", errors="replace") as f:
        for linea in reversed(f.readlines()):
            i = linea.find("RESULTADO: ")
            if i >= 0:
                try:
                    return json.loads(linea[i + len("RESULTADO: "):])
                except ValueError:
                    return None
    return None


def _fila(ruta, logs, origen, res=None, log=None):
    if res is None:
        with open(ruta, encoding="utf-8") as f:
            res = json.load(f)
    trans, eventos = res.get("transiciones"), res.get("eventos")
    log = log or res.get("log")
    if log and not os.path.isfile(log):
        cand = os.path.join(os.path.dirname(os.path.dirname(ruta)), "logs", os.path.basename(log))
        log = cand if os.path.isfile(cand) else None
    log = log or _log_cercano(ruta, logs)
    if (not trans or not eventos) and log:
        trans, eventos = _leer_log(log)
    trans, eventos = trans or [], eventos or []
    if not trans and not res.get("tiempo"):
        return None                         # JSON ajeno o vacio
    eventos_txt = "\n".join(eventos)
    config = res.get("config") or ""
    if not config:
        m = re.search(r"config (\S+)", eventos_txt)
        config = m.group(1) if m else ""
    args = res.get("args") or {}
    modo = args.get("modo") or ("directo" if "modo directo" in eventos_txt else
                                "guiado" if "guiado" in eventos_txt else "goal" if args.get("goal") else "")
    t_b = _t(trans, "LLEGADA_B", "ESCANEAR")
    t_volver = _t(trans, "VOLVER_A_A")
    t_anunciar = _t(trans, "ANUNCIAR")
    foto = res.get("foto")
    if foto:
        cand = os.path.join(os.path.dirname(os.path.dirname(ruta)), "logs", os.path.basename(foto))
        foto = cand if os.path.isfile(cand) else foto
    perfil = res.get("perfil") or {}
    fecha = res.get("fecha") or (_stamp(os.path.basename(ruta)).strftime("%Y-%m-%d %H:%M:%S")
                                 if _stamp(os.path.basename(ruta)) else "")
    fin = trans[-1] if trans else {}
    return log, dict(
        archivo=os.path.relpath(ruta, REPO).replace("\\", "/"), origen=origen, fecha=fecha,
        tarea=res.get("tarea", ""), modo=modo, config=os.path.basename(config),
        objetivo=res.get("objetivo") or args.get("persona") or "",
        ok=res.get("ok"), estado_final=res.get("estado", ""),
        motivo_final=fin.get("motivo", "") if res.get("estado") == "PARADA" else "",
        tiempo_s=res.get("tiempo"),
        t_ida_s=t_b,
        t_escaneo_s=round(t_volver - t_b, 2) if t_b is not None and t_volver is not None else None,
        t_regreso_s=round(t_anunciar - t_volver, 2) if t_volver is not None and t_anunciar is not None else None,
        choques=res.get("choques"),
        decisiones=len(res.get("decisiones") or []),
        callejones=eventos_txt.count("callejon sin caras"),
        rescates=eventos_txt.count("callejon aparente"),
        atascos=len(re.findall(r"atasco|escape", eventos_txt, re.I)),
        nombre=res.get("nombre") or "", score=res.get("score"), seguro=res.get("seguro"),
        acierto=(None if not (res.get("objetivo") or args.get("persona")) else
                 (res.get("nombre") or "").lower() == (res.get("objetivo") or args.get("persona") or "").lower()),
        qr=res.get("qr_objetivo") or "", hitos_qr=";".join(res.get("hitos_qr") or []),
        ciclo_ms_p95=perfil.get("ciclo_ms_p95"), cpu_pct=perfil.get("cpu_pct"),
        foto=os.path.relpath(foto, REPO).replace("\\", "/") if foto and os.path.isfile(foto) else (foto or ""),
        log=os.path.relpath(log, REPO).replace("\\", "/") if log else "",
    )


def _notas(salida, filas):
    """notas.csv se completa A MANO; aca solo se agregan las filas nuevas (vacias)."""
    ruta = os.path.join(salida, "notas.csv")
    notas = {}
    if os.path.isfile(ruta):
        with open(ruta, encoding="utf-8", newline="") as f:
            notas = {r["archivo"]: r for r in csv.DictReader(f)}
    for fl in filas:
        notas.setdefault(fl["archivo"], {c: "" for c in NOTAS_COLS} | {"archivo": fl["archivo"]})
    with open(ruta, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=NOTAS_COLS, extrasaction="ignore")
        w.writeheader()
        w.writerows(sorted(notas.values(), key=lambda r: r["archivo"]))
    return notas, ruta


def _md(v):
    if v is None or v == "":
        return "—"
    if isinstance(v, bool):
        return "sí" if v else "no"
    if isinstance(v, float):
        return f"{v:.2f}"
    return str(v).replace("|", "/")


def resumen(salida):
    fuentes = [(os.path.join(COMP, "resultados"), os.path.join(COMP, "logs"), "local")]
    for d in sorted(glob.glob(os.path.join(salida, "robot", "*", ""))):
        fuentes.append((os.path.join(d, "resultados"), os.path.join(d, "logs"), "robot:" + os.path.basename(d.rstrip("/\\"))))
    filas, vistos = [], set()
    for res_dir, log_dir, origen in fuentes:
        logs = sorted(glob.glob(os.path.join(log_dir, "mision_*.log")))
        for ruta in sorted(glob.glob(os.path.join(res_dir, "*.json"))):
            if os.path.basename(ruta) in vistos:
                continue                    # el mismo intento copiado en dos lados
            try:
                fl = _fila(ruta, logs, origen)
            except (OSError, ValueError) as e:
                print(f"[AVISO] {ruta}: {e}")
                continue
            if fl:
                usado, fl = fl
                vistos.add(os.path.basename(ruta))
                if usado:
                    vistos.add(os.path.basename(usado))
                filas.append(fl)
        for log in logs:                    # intentos que solo dejaron el .log
            if os.path.basename(log) in vistos:
                continue
            res = _res_de_log(log)
            if res is None:
                continue
            fl = _fila(log, logs, origen, res=res, log=log)
            if fl:
                vistos.add(os.path.basename(log))
                filas.append(fl[1])
    filas.sort(key=lambda f: f["fecha"])
    os.makedirs(salida, exist_ok=True)
    notas, ruta_notas = _notas(salida, filas)
    for fl in filas:
        n = notas.get(fl["archivo"], {})
        for c in NOTAS_COLS[1:]:
            fl[c] = n.get(c, "")
    if not filas:
        print("No hay intentos todavia (corre la mision o 'traer' desde el robot).")
        return
    ruta_csv = os.path.join(salida, "intentos.csv")
    with open(ruta_csv, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(filas[0]))
        w.writeheader()
        w.writerows(filas)

    L = [f"# Experimentación en el robot real — {datetime.now():%Y-%m-%d %H:%M}", "",
         "Generado por `scripts/recopilar_experimentos.py resumen`. Datos **medidos en el robot** "
         "(los del banco simulado van aparte, en `sim/`).", ""]
    for tarea in sorted({f["tarea"] for f in filas}):
        fs = [f for f in filas if f["tarea"] == tarea]
        L += [f"## Tarea {tarea}", ""]
        grupos = {}
        for f in fs:
            grupos.setdefault((f["modo"], f["config"]), []).append(f)
        L += ["| modo | config | intentos | completos | t total (mín / mediana) | choques | reconocimiento seguro |",
              "|---|---|---|---|---|---|---|"]
        for (modo, cfg), g in sorted(grupos.items()):
            ok = [f for f in g if f["ok"]]
            ts = [f["tiempo_s"] for f in ok if f["tiempo_s"]]
            tt = f"{min(ts):.1f} / {statistics.median(ts):.1f} s" if ts else "—"
            seg = sum(1 for f in g if f["seguro"])
            L.append(f"| {_md(modo)} | {_md(cfg)} | {len(g)} | {len(ok)} | {tt} | "
                     f"{sum(f['choques'] or 0 for f in g)} | {seg}/{len(g)} |")
        L += ["", "| fecha | config | ok | t total | ida | escaneo B | regreso | choques | decisiones | "
              "callejones | persona → reconocida (score) | QR | ancho | oficial | observaciones |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for f in fs:
            rec = f"{_md(f['objetivo'])} → {_md(f['nombre'])} ({_md(f['score'])}{'' if f['seguro'] else ', dudoso'})"
            estado = "sí" if f["ok"] else f"no ({f['estado_final']}{': ' + f['motivo_final'] if f['motivo_final'] else ''})"
            L.append(f"| {f['fecha']} | {_md(f['config'])} | {estado} | {_md(f['tiempo_s'])} | {_md(f['t_ida_s'])} | "
                     f"{_md(f['t_escaneo_s'])} | {_md(f['t_regreso_s'])} | {_md(f['choques'])} | {f['decisiones']} | "
                     f"{f['callejones']} | {rec} | {_md(f['qr'])} | {_md(f['ancho_pasillo_m'])} | "
                     f"{_md(f['oficial'])} | {_md(f['observaciones'])} |")
        fotos = [f for f in fs if f["foto"]]
        if fotos:
            L += ["", "Fotos del reconocimiento:", ""]
            L += [f"- {f['fecha']}: {f['nombre']} ({_md(f['score'])}) — `{f['foto']}`" for f in fotos]
        L.append("")
    sims = sorted(glob.glob(os.path.join(salida, "sim", "bench_*.csv")))
    if sims:
        L += ["## Banco simulado (último)", "", f"`{os.path.relpath(sims[-1], REPO)}` — **SIMULADO**, no mezclar con lo de arriba.", ""]
    estados = sorted(glob.glob(os.path.join(salida, "robot", "estado_*.json")))
    if estados:
        with open(estados[-1], encoding="utf-8") as f:
            e = json.load(f)
        L += ["## Último estado del robot", "", ", ".join(f"{k}: {e.get(k, '?')}" for k in
              ("fecha_pc", "bateria", "en_dock", "temp_c", "carga", "desfase_reloj_s")), ""]
    ruta_md = os.path.join(salida, "resumen.md")
    with open(ruta_md, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    print(f"{len(filas)} intentos -> {ruta_csv}\n  {ruta_md}\n  notas a mano: {ruta_notas}")


# --- sim -----------------------------------------------------------------------------------
CLAVES_BENCH = ("v_recto", "v_regreso", "v_acercar", "v_estrecho", "v_cerca", "w_giro", "k_giro",
                "d_lento", "max_accel", "arco_radio")


def sim(salida):
    os.makedirs(os.path.join(salida, "sim"), exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    filas = []
    for cfg in ("config.json", "config_rapido.json", "config_agresivo.json"):
        with open(os.path.join(COMP, cfg), encoding="utf-8") as f:
            d = json.load(f)
        perfil = ",".join(f"{k}={d[k]}" for k in CLAVES_BENCH if k in d) or "v_recto=0.26"
        tmp = os.path.join(salida, "sim", f".tmp_{cfg}.csv")
        print(f"banco simulado: {cfg} ...", flush=True)
        r = subprocess.run([sys.executable, os.path.join(COMP, "tests", "bench_directo.py"), "--perfil", perfil,
                            "--csv", tmp], cwd=COMP, capture_output=True, text=True)
        print("  " + (r.stdout.strip().splitlines() or ["(sin salida)"])[-2 if "csv:" in r.stdout else -1])
        if r.returncode != 0 or not os.path.isfile(tmp):
            print(r.stderr[-800:])
            continue
        with open(tmp, encoding="utf-8") as f:
            for row in csv.DictReader(f):
                row["perfil"] = cfg
                filas.append(row)
        os.remove(tmp)
    if filas:
        ruta = os.path.join(salida, "sim", f"bench_{stamp}.csv")
        with open(ruta, "w", encoding="utf-8", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(filas[0]))
            w.writeheader()
            w.writerows(filas)
        print(f"  -> {ruta} (SIMULADO)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("accion", choices=("traer", "estado", "resumen", "sim", "todo"))
    ap.add_argument("--ip", default=os.environ.get("TB4_IP", "192.168.185.3"))
    ap.add_argument("--carpeta", action="append", help="carpeta de la mision en el robot (repetible)")
    ap.add_argument("--salida", default=SALIDA)
    a = ap.parse_args()
    carpetas = a.carpeta or ["mision_directo"]
    if a.accion in ("traer", "todo"):
        traer(a.ip, carpetas, a.salida)
    if a.accion in ("estado", "todo"):
        estado(a.ip, a.salida)
    if a.accion in ("resumen", "todo"):
        resumen(a.salida)
    if a.accion == "sim":
        sim(a.salida)
        resumen(a.salida)


if __name__ == "__main__":
    main()
