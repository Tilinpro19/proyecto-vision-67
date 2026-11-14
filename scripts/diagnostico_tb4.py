"""
Diagnostico del TurtleBot 4 mientras navega: corre en la laptop (WSL, terminal conectada al robot)
junto a ver_mapa_tb4.py y deja logs para saber POR QUE no sigue la ruta.

Mide y registra:
  - frecuencia de /scan, /odom, /map y retraso de sus sellos de tiempo (reloj robot vs laptop)
  - edad de la transformada map->base_link (SLAM/tf atrasado = Pi saturada o red lenta)
  - /cmd_vel (y /cmd_vel_nav) pedido vs velocidad real en /odom (el robot obedece?)
  - meta (/goal_pose), plan (/plan) y estado de Nav2 (navigate_to_pose: EXECUTING/ABORTED...)
  - desviacion del robot respecto al plan (cross-track) y distancia a la meta
  - WARN/ERROR de Nav2 y SLAM en /rosout (agrupados, sin repetir spam)

Uso:
  python3 diagnostico_tb4.py            # Ctrl+C para terminar: imprime el resumen con conclusiones
  python3 diagnostico_tb4.py --dir ~/mis_logs

Salida: ~/diag_tb4/<fecha>/  series.csv (1 fila/s), eventos.log, resumen.txt
Solo LEE topics: no mueve ni configura nada en el robot.
"""
import argparse
import csv
import math
import os
import subprocess
import threading
import time
from collections import Counter, deque
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

STATUS = {1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
NAV_NODES = ("nav", "bt_", "controller", "planner", "costmap", "collision", "smoother",
             "behavior", "lifecycle", "slam", "docking", "waypoint")
MAP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class Rate:
    def __init__(self):
        self.t = deque(maxlen=300)
        self.n = 0
        self.last = None
        self.gap = 0.0        # mayor silencio desde la ultima lectura
        self.gap_all = 0.0    # mayor silencio de toda la sesion

    def tick(self):
        now = time.monotonic()
        if self.last is not None:
            g = now - self.last
            self.gap = max(self.gap, g)
            self.gap_all = max(self.gap_all, g)
        self.last = now
        self.t.append(now)
        self.n += 1

    def pop_gap(self):
        """Mayor silencio del ultimo periodo, contando el que sigue en curso."""
        g, self.gap = self.gap, 0.0
        if self.last is not None:
            g = max(g, time.monotonic() - self.last)
            self.gap_all = max(self.gap_all, g)
        return g

    def hz(self):
        now = time.monotonic()
        recent = [x for x in self.t if now - x <= 5.0]
        if len(recent) < 2 or now - recent[-1] > 3.0:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])


class Acc:
    """Media y maximo (en valor absoluto) sin guardar todas las muestras."""
    def __init__(self):
        self.n, self.sum, self.max = 0, 0.0, 0.0

    def add(self, x):
        self.n += 1
        self.sum += x
        self.max = max(self.max, abs(x))

    def mean(self):
        return self.sum / self.n if self.n else float("nan")


def dist_to_polyline(p, pts):
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


class Diag(Node):
    def __init__(self, outdir):
        super().__init__("diagnostico_tb4")
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.t0 = time.monotonic()
        self.evfile = open(os.path.join(outdir, "eventos.log"), "w", encoding="utf-8")
        self.csvfile = open(os.path.join(outdir, "series.csv"), "w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.csvfile)
        self.csv.writerow(["t_s", "x", "y", "yaw_deg", "cmd_v", "cmd_w", "odom_v", "odom_w",
                           "scan_hz", "odom_hz", "map_hz", "tf_age_s", "xtrack_m",
                           "dist_goal_m", "goal_activo", "scan_gap_s", "odom_gap_s", "ping_ms"])
        self.ping_ms = float("nan")
        self.ping_sent, self.ping_lost = 0, 0
        self.ping_rtt = Acc()
        self.stall = {"scan": False, "odom": False}
        self.cmd_nav = (0.0, 0.0)
        ip = os.environ.get("TB4_IP")
        if ip:
            threading.Thread(target=self._ping_loop, args=(ip,), daemon=True).start()

        self.rates = {k: Rate() for k in ("scan", "odom", "map", "plan", "cmd")}
        self.lat = {"scan": Acc(), "odom": Acc()}
        self.tf_age, self.xtrack = Acc(), Acc()
        self.tf_fail = 0
        self.cmd = (0.0, 0.0)
        self.odom = (0.0, 0.0)
        self.plan = []
        self.goal = None
        self.goal_active = False
        self.goal_status = {}
        self.status_count = Counter()
        self.stuck_s = 0
        self.stuck_events = 0
        self.log_counts = Counter()
        self.log_first = {}
        self.dropped = 0
        self.last_pose = None

        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", lambda m: self.rates["map"].tick(), MAP_QOS)
        self.create_subscription(Path, "/plan", self._plan, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._goal, 10)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                 self._status, STATUS_QOS)
        self.create_subscription(Log, "/rosout", self._rosout, 100)
        for topic in ("/cmd_vel", "/cmd_vel_nav"):
            self._sub_cmd(topic)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.create_timer(1.0, self._tick)
        self.ev("Diagnostico iniciado. Logs en " + outdir)

    # --- utilidades ---------------------------------------------------------
    def now_s(self):
        return time.monotonic() - self.t0

    def ev(self, msg):
        line = f"[{self.now_s():7.1f}s] {msg}"
        print(line, flush=True)
        self.evfile.write(line + "\n")
        self.evfile.flush()

    def _sub_cmd(self, topic):
        msg_type = Twist
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            types = dict(self.get_topic_names_and_types()).get(topic)
            if types:
                msg_type = TwistStamped if "TwistStamped" in types[0] else Twist
                break
        self.create_subscription(msg_type, topic, lambda m, t=topic: self._cmd(m, t), 10)

    def _ping_loop(self, ip):
        """Un ping cada 2 s en segundo plano: calidad del enlace laptop-robot."""
        while True:
            self.ping_sent += 1
            try:
                out = subprocess.run(["ping", "-c", "1", "-W", "1", ip], capture_output=True,
                                     text=True, timeout=3).stdout
                rtt = float(out.split("time=")[1].split()[0].rstrip("ms"))
                self.ping_ms = rtt
                self.ping_rtt.add(rtt)
            except Exception:
                self.ping_ms = float("nan")
                self.ping_lost += 1
            time.sleep(2.0)

    def _stamp_lag(self, stamp):
        n = self.get_clock().now().nanoseconds
        return n / 1e9 - (stamp.sec + stamp.nanosec / 1e9)

    # --- callbacks ----------------------------------------------------------
    def _scan(self, m):
        self.rates["scan"].tick()
        self.lat["scan"].add(self._stamp_lag(m.header.stamp))

    def _odom(self, m):
        self.rates["odom"].tick()
        self.lat["odom"].add(self._stamp_lag(m.header.stamp))
        self.odom = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def _cmd(self, m, topic):
        tw = m.twist if hasattr(m, "twist") else m
        if topic == "/cmd_vel":
            self.rates["cmd"].tick()
            self.cmd = (tw.linear.x, tw.angular.z)
        else:
            self.cmd_nav = (tw.linear.x, tw.angular.z)

    def _plan(self, m):
        self.rates["plan"].tick()
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def _goal(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y)
        self.ev(f"META recibida en /goal_pose: x={self.goal[0]:.2f} y={self.goal[1]:.2f}")

    def _status(self, m):
        active = False
        for s in m.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()[:8]
            if self.goal_status.get(gid) != s.status:
                self.goal_status[gid] = s.status
                name = STATUS.get(s.status, str(s.status))
                self.status_count[name] += 1
                self.ev(f"Nav2 navigate_to_pose [{gid}] -> {name}")
            active = active or s.status in (1, 2, 3)
        self.goal_active = active

    def _rosout(self, m):
        if m.level < 30 or not any(k in m.name.lower() for k in NAV_NODES):
            return
        if "Message Filter dropping" in m.msg:
            self.dropped += 1
            return
        key = (m.name, m.msg[:90])
        self.log_counts[key] += 1
        if key not in self.log_first:
            self.log_first[key] = self.now_s()
            lvl = "ERROR" if m.level >= 40 else "WARN"
            self.ev(f"/rosout {lvl} {m.name}: {m.msg[:140]}")

    # --- muestreo 1 Hz ------------------------------------------------------
    def _pose(self):
        try:
            t = self.tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            self.tf_fail += 1
            return None, float("nan")
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        age = self._stamp_lag(t.header.stamp)
        self.tf_age.add(age)
        return (t.transform.translation.x, t.transform.translation.y, yaw), age

    def _tick(self):
        pose, age = self._pose()
        xt = dg = float("nan")
        if pose:
            self.last_pose = pose
            if self.goal_active and len(self.plan) > 1:
                xt = dist_to_polyline(pose[:2], self.plan)
                self.xtrack.add(xt)
            if self.goal:
                dg = math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
        # el robot obedece? pide moverse pero la odometria no se mueve
        asked = abs(self.cmd[0]) > 0.05 or abs(self.cmd[1]) > 0.2
        moving = abs(self.odom[0]) > 0.01 or abs(self.odom[1]) > 0.05
        self.stuck_s = self.stuck_s + 1 if asked and not moving and self.rates["cmd"].hz() > 0 else 0
        if self.stuck_s == 3:
            self.stuck_events += 1
            self.ev("cmd_vel pide movimiento pero /odom no se mueve (>3 s): dock/ruedas/Create 3?")

        r = {k: v.hz() for k, v in self.rates.items()}
        f = lambda x: "" if x != x else f"{x:.3f}"
        gaps = {k: self.rates[k].pop_gap() for k in ("scan", "odom")}
        for k, g in gaps.items():
            if g > 2.0 and not self.stall[k]:
                self.stall[k] = True
                self.ev(f"/{k} CONGELADO: sin datos {g:.1f} s (red cortada o robot colgado)")
            elif g <= 2.0 and self.stall[k]:
                self.stall[k] = False
                self.ev(f"/{k} volvio")
        p = pose or (float("nan"),) * 3
        self.csv.writerow([f"{self.now_s():.1f}", f(p[0]), f(p[1]), f(math.degrees(p[2])),
                           f"{self.cmd[0]:.3f}", f"{self.cmd[1]:.3f}",
                           f"{self.odom[0]:.3f}", f"{self.odom[1]:.3f}",
                           f"{r['scan']:.1f}", f"{r['odom']:.1f}", f"{r['map']:.2f}",
                           f(age), f(xt), f(dg), int(self.goal_active),
                           f"{gaps['scan']:.2f}", f"{gaps['odom']:.2f}", f(self.ping_ms)])
        self.csvfile.flush()
        print(f"[{self.now_s():6.0f}s] scan {r['scan']:4.1f}Hz odom {r['odom']:4.1f}Hz "
              f"tf_edad {(f(age) + 's') if age == age else 'SIN TF'} cmd v={self.cmd[0]:+.2f} w={self.cmd[1]:+.2f} "
              f"odom v={self.odom[0]:+.2f} xtrack {f(xt) or '-'}m goal {'SI' if self.goal_active else 'no'}",
              flush=True)

    # --- resumen ------------------------------------------------------------
    def summary(self):
        L = ["", "=== RESUMEN DE DIAGNOSTICO ===", f"Duracion: {self.now_s():.0f} s"]
        hz = {k: v.hz() for k, v in self.rates.items()}
        L.append(f"/scan {hz['scan']:.1f} Hz | /odom {hz['odom']:.1f} Hz | /map {hz['map']:.2f} Hz | "
                 f"/plan: {self.rates['plan'].n} mensajes")
        L.append(f"Retraso de sello scan: media {self.lat['scan'].mean():.2f} s (max {self.lat['scan'].max:.2f}) | "
                 f"odom: media {self.lat['odom'].mean():.2f} s")
        off = self.lat["scan"].mean()
        off = off if abs(off) > 1.0 else 0.0   # desfase de reloj robot-laptop; se resta para ver el retraso real
        tf_rel = self.tf_age.mean() - off
        if off:
            L.append(f"Desfase de reloj robot-laptop: {off:.0f} s (~{off / 86400:.1f} dias)")
        L.append(f"Edad tf map->base_link (sin desfase): media {tf_rel:.2f} s, "
                 f"fallos de lookup: {self.tf_fail}")
        L.append(f"Mayor silencio: scan {self.rates['scan'].gap_all:.1f} s | odom {self.rates['odom'].gap_all:.1f} s")
        if self.ping_sent:
            loss = 100.0 * self.ping_lost / self.ping_sent
            L.append(f"Ping al robot: {self.ping_sent} enviados, {loss:.0f}% perdidos, "
                     f"rtt medio {self.ping_rtt.mean():.0f} ms (max {self.ping_rtt.max:.0f})")
        L.append(f"Estados Nav2: {dict(self.status_count) or 'ninguno'}")
        L.append(f"Desviacion al plan: media {self.xtrack.mean():.2f} m (max {self.xtrack.max:.2f})")
        L.append(f"Mensajes 'Message Filter dropping': {self.dropped}")
        for (name, msg), n in self.log_counts.most_common(8):
            L.append(f"  x{n} {name}: {msg}")
        L += ["", "--- Conclusiones probables ---"]
        c = []
        if self.rates["scan"].n == 0:
            c.append("No llego /scan: red/DDS o bringup. Revisa la conexion.")
        elif hz["scan"] < 5:
            c.append(f"/scan llega a {hz['scan']:.1f} Hz (esperado ~10): red lenta o robot saturado.")
        lag = self.lat["scan"].mean()
        if lag == lag and abs(lag) > 1.0:
            c.append(f"Relojes desfasados ~{lag:.0f} s (~{lag / 86400:.1f} dias) entre robot y laptop: iguala la hora (guia, seccion de relojes).")
        if self.tf_fail > 5:
            c.append(f"Fallo al leer map->base_link {self.tf_fail} veces: SLAM no corre o tf atrasada.")
        elif self.tf_age.n and tf_rel > 0.5:
            c.append(f"tf map->base_link llega con {tf_rel:.2f} s de retraso: Pi saturada o red lenta.")
        if self.ping_sent >= 5 and 100.0 * self.ping_lost / self.ping_sent > 2:
            c.append(f"El enlace pierde {100.0 * self.ping_lost / self.ping_sent:.0f}% de los pings: Wi-Fi inestable; prueba cable o acercate al router.")
        elif self.ping_rtt.n and self.ping_rtt.max > 200:
            c.append(f"Ping con picos de {self.ping_rtt.max:.0f} ms: enlace con latencia irregular (Wi-Fi saturada).")
        gs, go = self.rates["scan"].gap_all, self.rates["odom"].gap_all
        if max(gs, go) > 2.0 and self.rates["scan"].n:
            c.append(f"Hubo congelamientos de datos (scan {gs:.1f} s, odom {go:.1f} s): corta de red o Pi colgada. "
                     "Cruza con diag_robot.sh (CPU/temperatura del robot) en ese instante.")
        if self.dropped > 20:
            c.append(f"{self.dropped} 'Message Filter dropping': el robot no procesa el lidar a tiempo (CPU) o tf atrasada.")
        if self.status_count["ABORTED"]:
            c.append(f"Nav2 aborto {self.status_count['ABORTED']} meta(s): mira los /rosout de arriba (planner/controller).")
        if self.stuck_events:
            c.append(f"{self.stuck_events} vez(veces) cmd_vel pidio mover y la odometria no se movio: el robot no obedece "
                     "(en el dock, ruedas, o Create 3 frenado por colision/seguro).")
        if self.xtrack.n and self.xtrack.mean() > 0.3:
            c.append(f"Se desvia {self.xtrack.mean():.2f} m del plan en promedio: controlador o localizacion inestable.")
        if self.goal and not self.status_count and not self.rates["plan"].n:
            c.append("Se envio una meta pero Nav2 no reacciono (sin /plan ni estados): Nav2 no esta corriendo o no ve /goal_pose.")
        L += ["- " + x for x in c] or ["- Sin anomalias claras en esta sesion."]
        return "\n".join(L)

    def close(self):
        text = self.summary()
        print(text)
        with open(os.path.join(self.outdir, "resumen.txt"), "w", encoding="utf-8") as fh:
            fh.write(text + "\n")
        self.evfile.close()
        self.csvfile.close()
        print(f"\nLogs guardados en {self.outdir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.expanduser("~/diag_tb4"), help="carpeta base de logs")
    args = ap.parse_args()
    outdir = os.path.join(args.dir, datetime.now().strftime("%Y%m%d_%H%M%S"))
    rclpy.init()
    node = Diag(outdir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
