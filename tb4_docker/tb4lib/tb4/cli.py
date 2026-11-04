"""Comando `tb4`: reemplaza tb4_check.sh, chequeo_reloj_tb4.py, ver_camara_tb4.py,
ver_lidar_tb4.py y teleop_wasd.py.

  tb4 info                 entorno ROS/DDS que se va a usar
  tb4 check                topicos clave, frecuencias, bumper y reloj
  tb4 reloj [--sync]       desfase PC / Pi / Create 3 (y fijar la hora de la Pi)
  tb4 camara               ventana con la OAK-D (comprimida) y FPS
  tb4 lidar                vista cenital del LiDAR
  tb4 teleop               W/A/S/D, espacio = parar, q = salir
  tb4 mover V W SEG        velocidad fija durante SEG segundos
  tb4 avanzar M | girar G  primitivas con odometria (m / grados)
"""
import argparse
import math
import os
import sys
import time


def _robot(args, **kw):
    from .robot import Robot
    return Robot(ip=args.ip, domain_id=args.domain, modo=args.modo, **kw)


def cmd_info(args):
    from .conexion import configurar_entorno
    modo = configurar_entorno(args.ip, args.domain, args.modo)
    for k in ("TB4_IP", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_STATIC_PEERS",
              "FASTRTPS_DEFAULT_PROFILES_FILE"):
        print(f"{k:32s} {os.environ.get(k, '-')}")
    print(f"{'modo':32s} {modo}")


def cmd_check(args):
    ip = args.ip or os.environ.get("TB4_IP")
    if ip:
        ok = os.system(f"ping -c 2 -W 2 {ip} >/dev/null 2>&1") == 0
        print(f"[{'ok' if ok else 'FALLA'}] ping {ip}")
        if not ok:
            return 1
    with _robot(args, dry=True) as bot:
        cuenta = {"scan": 0, "odom": 0, "camara": 0}
        t0, prev = time.monotonic(), {"scan": 0.0, "odom": 0.0, "camara": 0.0}
        print("[..] escuchando 5 s...")
        while time.monotonic() - t0 < 5:
            for k, s in (("scan", bot.lidar), ("odom", bot.odom), ("camara", bot.camara)):
                if s.last_t != prev[k]:
                    prev[k] = s.last_t
                    cuenta[k] += 1
            if bot.camara.msg is not None:
                bot.camara.latest()
            time.sleep(0.005)
        falt = bot.faltantes()
        print(f"[{'ok' if not falt else 'AVISO'}] topicos clave"
              + (f": faltan {', '.join(falt)}" if falt else ""))
        for k, esperado in (("scan", 5), ("odom", 10), ("camara", 3)):
            hz = cuenta[k] / 5
            print(f"[{'ok' if hz >= esperado else 'FALLA'}] {k:7s} {hz:5.1f} Hz")
        print(f"[{'ok' if bot.bumper.disponible else '--'}] bumper (irobot_create_msgs)")
        _imprimir_reloj(bot.desfase_reloj())
    return 0


def _imprimir_reloj(d):
    for k, txt in (("pc_pi", "PC - Pi (/scan)"), ("pc_create3", "PC - Create 3 (/odom)"),
                   ("pi_create3", "Pi - Create 3")):
        v = d[k]
        if v is None:
            print(f"[--]   {txt}: sin datos")
        else:
            estado = "ok" if abs(v) < (0.5 if k == "pi_create3" else 2.0) else "AVISO"
            print(f"[{estado}] {txt}: {v:+.2f} s")
    if d["pi_create3"] is not None and abs(d["pi_create3"]) >= 0.5:
        print("       Create 3 desfasado de la Pi: SLAM descartara los /scan. Espera unos minutos o reinicia turtlebot4.service en el robot")


def cmd_reloj(args):
    if args.sync:
        from .reloj import sincronizar_hora
        return sincronizar_hora(args.ip)
    with _robot(args, dry=True, camara=False) as bot:
        _imprimir_reloj(bot.desfase_reloj())
    return 0


def cmd_camara(args):
    import cv2
    with _robot(args, dry=True, topic_camara=args.topic) as bot:
        n, t0, fps = 0, time.monotonic(), 0.0
        while True:
            img = bot.camara.nueva()
            if img is not None:
                n += 1
                if time.monotonic() - t0 >= 1:
                    fps, n, t0 = n / (time.monotonic() - t0), 0, time.monotonic()
                img = img.copy()
                cv2.putText(img, f"{fps:.1f} FPS", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("tb4 camara", img)
            if cv2.waitKey(10) & 0xFF in (ord("q"), 27):
                break
    return 0


def cmd_lidar(args):
    import cv2
    import numpy as np
    size, rango = 600, args.rango
    with _robot(args, dry=True, camara=False) as bot:
        while True:
            img = np.zeros((size, size, 3), np.uint8)
            c, esc = size // 2, size / (2 * rango)
            for r in range(1, int(rango) + 1):
                cv2.circle(img, (c, c), int(r * esc), (60, 60, 60), 1)
            xy = bot.lidar.puntos_xy()
            if len(xy):
                px = (c - xy[:, 1] * esc).astype(int)   # y (izquierda) -> pantalla izquierda
                py = (c - xy[:, 0] * esc).astype(int)   # x (frente)    -> arriba
                ok = (px >= 0) & (px < size) & (py >= 0) & (py < size)
                img[py[ok], px[ok]] = (0, 255, 255)
            cv2.circle(img, (c, c), int(0.17 * esc), (255, 255, 255), 1)
            cv2.line(img, (c, c), (c, c - int(0.3 * esc)), (0, 255, 0), 2)
            d = bot.lidar.distancia(0.0)
            col = (0, 0, 255) if d is not None and d < 0.4 else (0, 255, 0)
            cv2.putText(img, f"frente: {d:.2f} m" if d else "sin datos", (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            cv2.imshow("tb4 lidar", img)
            if cv2.waitKey(50) & 0xFF in (ord("q"), 27):
                break
    return 0


def cmd_teleop(args):
    import select
    import termios
    import tty
    from .limites import LIMIT_ANGULAR, LIMIT_LINEAR, STEP_ANGULAR, STEP_LINEAR, clamp
    teclas = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1)}
    v, w = args.vel, args.ang
    old = termios.tcgetattr(sys.stdin)
    with _robot(args, camara=False) as bot:
        print("W/A/S/D mover (frena al soltar) | u/j vel lineal | i/k vel angular | espacio parar | q salir")
        try:
            tty.setcbreak(sys.stdin.fileno())
            ultimo = 0.0
            while True:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    k = sys.stdin.read(1).lower()
                    if k == "q":
                        break
                    if k in teclas:
                        a, b = teclas[k]
                        bot.mover(a * v, b * w)
                        ultimo = time.monotonic()
                    elif k == " ":
                        bot.parar()
                    elif k in "ujik":
                        v = clamp(v + {"u": STEP_LINEAR, "j": -STEP_LINEAR}.get(k, 0), 0.05, LIMIT_LINEAR)
                        w = clamp(w + {"i": STEP_ANGULAR, "k": -STEP_ANGULAR}.get(k, 0), 0.2, LIMIT_ANGULAR)
                        print(f"\rlineal {v:.2f} m/s | angular {w:.2f} rad/s   ", end="", flush=True)
                elif ultimo and time.monotonic() - ultimo > 0.25:
                    bot.parar()
                    ultimo = 0.0
                elif ultimo:
                    bot.mover(*bot.cmd.last)   # mantener mientras la tecla se repite
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    return 0


def cmd_mover(args):
    with _robot(args, camara=False) as bot:
        bot.mover(args.v, args.w, args.seg)
    return 0


def cmd_avanzar(args):
    with _robot(args, camara=False) as bot:
        return 0 if bot.avanzar(args.metros) else 1


def cmd_girar(args):
    with _robot(args, camara=False) as bot:
        return 0 if bot.girar(math.radians(args.grados)) else 1


def main(argv=None):
    from .limites import START_ANGULAR, START_LINEAR, TOPIC_CAMARA
    ap = argparse.ArgumentParser(prog="tb4", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", help="IP del robot (por defecto TB4_IP)")
    ap.add_argument("--domain", type=int, help="ROS_DOMAIN_ID (por defecto el del entorno)")
    ap.add_argument("--modo", default="auto", choices=("auto", "multicast", "peers"))
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info").set_defaults(fn=cmd_info)
    sub.add_parser("check").set_defaults(fn=cmd_check)
    p = sub.add_parser("reloj"); p.add_argument("--sync", action="store_true"); p.set_defaults(fn=cmd_reloj)
    p = sub.add_parser("camara"); p.add_argument("--topic", default=TOPIC_CAMARA); p.set_defaults(fn=cmd_camara)
    p = sub.add_parser("lidar"); p.add_argument("--rango", type=float, default=4.0); p.set_defaults(fn=cmd_lidar)
    p = sub.add_parser("teleop")
    p.add_argument("--vel", type=float, default=START_LINEAR)
    p.add_argument("--ang", type=float, default=START_ANGULAR)
    p.set_defaults(fn=cmd_teleop)
    p = sub.add_parser("mover")
    p.add_argument("v", type=float); p.add_argument("w", type=float); p.add_argument("seg", type=float)
    p.set_defaults(fn=cmd_mover)
    p = sub.add_parser("avanzar"); p.add_argument("metros", type=float); p.set_defaults(fn=cmd_avanzar)
    p = sub.add_parser("girar"); p.add_argument("grados", type=float); p.set_defaults(fn=cmd_girar)
    args = ap.parse_args(argv)
    try:
        sys.exit(args.fn(args) or 0)
    except TimeoutError as e:
        sys.exit(f"[FALLA] {e}")
    except KeyboardInterrupt:
        sys.exit(130)


if __name__ == "__main__":
    main()
