"""
Base 0 de la Tarea A: revisa los sensores que usa navegacion.py y graba un scan real.

- /scan: frecuencia, puntos validos y distancia al frente / izquierda / atras / derecha
  (con la rotacion --lidar-rot ya aplicada: compara con lo que ves alrededor del robot).
- /odom: que llegue y la pose actual.
- /cmd_vel: tipo (Twist o TwistStamped). No publica nada: el robot no se mueve.
- Guarda el ultimo scan en 1_competencia/scans/<nombre>.npz (ranges, angles en el marco
  del robot) para las pruebas de 1_competencia/tests.

Uso (con tb4_connect.sh ya cargado):
    python3 grabar_scan.py pasillo_cajas --secs 5
"""
import argparse
import math
import os
import time

import numpy as np

SCANS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "1_competencia", "scans")


def main():
    ap = argparse.ArgumentParser(description="Revisa /scan, /odom, /cmd_vel y graba un scan")
    ap.add_argument("nombre", nargs="?", default=time.strftime("scan_%H%M%S"))
    ap.add_argument("--scan", default="/scan")
    ap.add_argument("--odom", default="/odom")
    ap.add_argument("--cmd", default="/cmd_vel")
    ap.add_argument("--lidar-rot", type=float, default=90.0,
                    help="grados a sumar al /scan para que 0 sea el frente (TB4: 90)")
    ap.add_argument("--secs", type=float, default=5.0, help="segundos escuchando")
    args = ap.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("grabar_scan")
    rot = math.radians(args.lidar_rot)
    scans, odoms = [], []
    node.create_subscription(LaserScan, args.scan,
                             lambda m: scans.append((time.monotonic(), m)), qos_profile_sensor_data)
    node.create_subscription(Odometry, args.odom,
                             lambda m: odoms.append((time.monotonic(), m)), qos_profile_sensor_data)
    try:
        print(f"Escuchando {args.secs:.0f} s...")
        end = time.monotonic() + args.secs
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

        types = dict(node.get_topic_names_and_types())
        print(f"\n{args.cmd}: {types.get(args.cmd, ['NO APARECE'])[0]}")

        if odoms:
            p = odoms[-1][1].pose.pose
            yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                             1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
            print(f"{args.odom}: {len(odoms) / args.secs:.1f} Hz | pose x={p.position.x:.2f} "
                  f"y={p.position.y:.2f} yaw={math.degrees(yaw):.0f} grados")
        else:
            print(f"{args.odom}: NO LLEGA")

        if not scans:
            raise SystemExit(f"{args.scan}: NO LLEGA. Revisa tb4_connect.sh y el bringup.")
        t = np.array([s[0] for s in scans])
        gaps = np.diff(t) if len(t) > 1 else np.array([0.0])
        print(f"{args.scan}: {len(scans) / args.secs:.1f} Hz | hueco maximo entre scans "
              f"{gaps.max() * 1000:.0f} ms")

        m = scans[-1][1]
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + rot
        a = np.arctan2(np.sin(a), np.cos(a))
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        print(f"Ultimo scan: {ok.sum()}/{len(r)} puntos validos, rango {m.range_min:.2f}-{m.range_max:.1f} m")

        print("\nDistancia libre (percentil 20 en +-10 grados). Comparala con lo que ves:")
        for name, deg in (("FRENTE", 0), ("IZQUIERDA", 90), ("ATRAS", 180), ("DERECHA", -90)):
            d = np.abs(np.arctan2(np.sin(a[ok] - math.radians(deg)), np.cos(a[ok] - math.radians(deg))))
            sel = r[ok][d < math.radians(10)]
            txt = f"{np.percentile(sel, 20):.2f} m" if len(sel) else "nada"
            print(f"  {name:10s} {txt}")

        os.makedirs(SCANS_DIR, exist_ok=True)
        path = os.path.join(SCANS_DIR, args.nombre + ".npz")
        np.savez(path, ranges=r, angles=a, lidar_rot=args.lidar_rot)
        print(f"\nGuardado: {os.path.normpath(path)}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
