"""
Calibracion: corre N intentos de mision.py por cada juego de parametros, registra
tiempo, choques (bumper del Create 3 o distancia minima del LiDAR, los cuenta mision.py)
y nombre, y ordena por costo = t + c * choques + e * nombre_erroneo.
Un intento fallido (no volvio o supero t_max) cuesta t_max + c * choques.

Uso (WSL, tras 'source tb4_connect.sh ...'; todo lo que va despues de -- es para mision.py):
  python3 calibrar.py --intentos 3 --c 5 --esperado Juan -- --tarea A --goal 3 0
  python3 calibrar.py --barrido max_linear=0.15,0.2,0.25 --barrido safety_margin=0.08,0.1 \\
      --intentos 2 --c 5 -- --tarea A --goal 3 0

Entre intentos pide poner el robot en A y pulsar Enter (aqui si se usa teclado: esto es
laboratorio, no competencia). Resultados en 2_laboratorio/calibracion/*.csv.
"""
import argparse
import csv
import itertools
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MISION = os.path.join(os.path.dirname(HERE), "1_competencia", "mision.py")


def costo(res, c, e=0.0, esperado=None, t_max=300.0):
    """Tiempo efectivo de un intento (menor = mejor)."""
    choques = res.get("choques", 0)
    if not res.get("ok"):
        return t_max + c * choques
    malo = esperado is not None and res.get("nombre") != esperado
    return res["tiempo"] + c * choques + e * malo


def combinaciones(barridos):
    """['a=1,2', 'b=3'] -> [{'a': '1', 'b': '3'}, {'a': '2', 'b': '3'}]"""
    ejes = []
    for b in barridos:
        k, _, vals = b.partition("=")
        ejes.append([(k.strip(), v.strip()) for v in vals.split(",") if v.strip()])
    return [dict(c) for c in itertools.product(*ejes)] if ejes else [{}]


def resumen(filas):
    """Por configuracion: exito, costo medio y mejor. Ordenado por costo medio."""
    grupos = {}
    for f in filas:
        grupos.setdefault(f["config"], []).append(f)
    out = []
    for cfg, fs in grupos.items():
        costos = [f["costo"] for f in fs]
        out.append(dict(config=cfg, intentos=len(fs), exitos=sum(f["ok"] for f in fs),
                        costo_medio=sum(costos) / len(costos), mejor=min(costos),
                        choques=sum(f["choques"] for f in fs)))
    return sorted(out, key=lambda r: r["costo_medio"])


def main():
    argv = sys.argv[1:]
    extra = argv[argv.index("--") + 1:] if "--" in argv else []
    argv = argv[:argv.index("--")] if "--" in argv else argv
    ap = argparse.ArgumentParser(description="Calibracion de parametros por intentos")
    ap.add_argument("--intentos", type=int, default=3, help="intentos por configuracion")
    ap.add_argument("--c", type=float, default=5.0, help="s de penalidad por choque")
    ap.add_argument("--e", type=float, default=30.0, help="s de penalidad por nombre erroneo")
    ap.add_argument("--esperado", help="nombre correcto de la foto (para contar errores)")
    ap.add_argument("--t-max", type=float, default=300.0)
    ap.add_argument("--barrido", action="append", default=[], metavar="param=v1,v2,...")
    args = ap.parse_args(argv)
    if "--tarea" not in extra:
        extra = ["--tarea", "A"] + extra

    out_dir = os.path.join(HERE, "calibracion")
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, f"calib_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    filas = []
    campos = ["config", "intento", "ok", "tiempo", "choques", "nombre", "seguro", "costo"]
    with open(csv_path, "w", newline="", encoding="utf-8") as fcsv:
        w = csv.DictWriter(fcsv, fieldnames=campos)
        w.writeheader()
        try:
            for combo in combinaciones(args.barrido):
                nombre_cfg = " ".join(f"{k}={v}" for k, v in combo.items()) or "base"
                for i in range(1, args.intentos + 1):
                    r = input(f"\n[{nombre_cfg}] intento {i}/{args.intentos}: pon el robot en A "
                              "y pulsa Enter (s = saltar config, q = terminar) ").strip().lower()
                    if r == "q":
                        raise KeyboardInterrupt
                    if r == "s":
                        break
                    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tmp:
                        res_path = tmp.name
                    cmd = [sys.executable, MISION, *extra, "--resultado", res_path,
                           "--param", f"t_max={args.t_max}"]
                    for k, v in combo.items():
                        cmd += ["--param", f"{k}={v}"]
                    try:
                        subprocess.run(cmd, cwd=os.path.dirname(MISION))
                    except KeyboardInterrupt:
                        pass  # Ctrl+C corta el intento; mision.py igual guarda el resultado
                    try:
                        with open(res_path, encoding="utf-8") as f:
                            res = json.load(f)
                    except (OSError, ValueError):
                        res = dict(ok=False, tiempo=args.t_max, choques=0)
                    finally:
                        if os.path.exists(res_path):
                            os.remove(res_path)
                    fila = dict(config=nombre_cfg, intento=i, ok=bool(res.get("ok")),
                                tiempo=res.get("tiempo"), choques=res.get("choques", 0),
                                nombre=res.get("nombre"), seguro=res.get("seguro"),
                                costo=round(costo(res, args.c, args.e, args.esperado, args.t_max), 2))
                    filas.append(fila)
                    w.writerow(fila)
                    fcsv.flush()
                    print(f"-> ok={fila['ok']} t={fila['tiempo']} choques={fila['choques']} "
                          f"nombre={fila['nombre']} costo={fila['costo']}")
        except KeyboardInterrupt:
            print("\nCalibracion interrumpida.")

    print(f"\nResultados: {csv_path}\n")
    print(f"{'config':40s} {'exito':>6s} {'costo medio':>12s} {'mejor':>8s} {'choques':>8s}")
    for r in resumen(filas):
        print(f"{r['config']:40s} {r['exitos']:>3d}/{r['intentos']:<2d} "
              f"{r['costo_medio']:12.1f} {r['mejor']:8.1f} {r['choques']:8d}")


if __name__ == "__main__":
    main()
