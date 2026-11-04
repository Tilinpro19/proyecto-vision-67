"""Fachada: un objeto Robot con nodo, sensores, /cmd_vel y primitivas de movimiento."""
import math
import threading
import time

from .anuncio import Anunciador
from .conexion import configurar_entorno, topicos
from .limites import (LIDAR_ROT_DEG, TOPIC_CAMARA, TOPIC_CMD_VEL, TOPIC_ODOM, TOPIC_SCAN,
                      TOPICOS_CLAVE)
from .movimiento import Sender
from .reloj import medir_desfase
from .sensores import Bumper, Camara, Lidar, Odom

HZ = 20.0  # el Create 3 frena si deja de recibir /cmd_vel; publicamos a 20 Hz


class Robot:
    """
    Robot(ip=None, domain_id=None)       -> toma TB4_IP / ROS_DOMAIN_ID del entorno
    spin='hilo'   : el nodo gira solo en un hilo (lo comodo en scripts y notebooks)
    spin='manual' : tu bucle llama bot.spin() (como mision.py)
    camara=False  : no suscribirse a la OAK-D (ahorra Wi-Fi)
    dry=True      : no publica velocidades

    Siempre frena al salir (with / cerrar() / Ctrl+C).
    """

    def __init__(self, ip=None, domain_id=None, modo="auto", nombre="tb4_pc", spin="hilo",
                 camara=True, dry=False, scan=TOPIC_SCAN, odom=TOPIC_ODOM,
                 cmd_vel=TOPIC_CMD_VEL, topic_camara=TOPIC_CAMARA, lidar_rot=LIDAR_ROT_DEG,
                 tipo_cmd=None):
        import rclpy

        configurar_entorno(ip, domain_id, modo)
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(nombre)
        self.lidar = Lidar(self.node, scan, lidar_rot)
        self.odom = Odom(self.node, odom)
        self.camara = Camara(self.node, topic_camara) if camara else None
        self.bumper = Bumper(self.node)
        self._exec = None
        if spin == "hilo":
            from rclpy.executors import MultiThreadedExecutor
            self._exec = MultiThreadedExecutor()
            self._exec.add_node(self.node)
            threading.Thread(target=self._girar_hilo, daemon=True).start()
        self.cmd = Sender(self.node, cmd_vel, dry, tipo=tipo_cmd)
        self._anunciador = None

    # --- ciclo de vida -------------------------------------------------------------
    def _girar_hilo(self):
        try:
            self._exec.spin()
        except Exception:  # contexto cerrado al salir
            pass

    def spin(self, timeout=0.05):
        """Solo en spin='manual'."""
        if self._exec is None:
            self.rclpy.spin_once(self.node, timeout_sec=timeout)
        else:
            time.sleep(timeout)

    def cerrar(self):
        try:
            self.cmd.stop()
        finally:
            if self._exec is not None:
                self._exec.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            self.rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.cerrar()

    # --- estado --------------------------------------------------------------------
    def esperar_sensores(self, timeout=15.0, camara=False):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.lidar.fresh() and self.odom.fresh() and (
                    not camara or (self.camara and self.camara.latest() is not None)):
                return True
            self.spin(0.05)
        raise TimeoutError("No llegan /scan y /odom: revisa la conexion (tb4 check)")

    def topicos(self):
        return topicos(self.node)

    def faltantes(self):
        vis = self.topicos()
        return [t for t in TOPICOS_CLAVE if t not in vis]

    def desfase_reloj(self):
        return medir_desfase(self.node)

    @property
    def pose(self):
        return self.odom.pose

    # --- movimiento ----------------------------------------------------------------
    def mover(self, lineal, angular, duracion=None):
        """Publica una velocidad. Con duracion (s) la sostiene a 20 Hz y luego frena."""
        if duracion is None:
            self.cmd.send(lineal, angular)
            return
        end = time.monotonic() + duracion
        while time.monotonic() < end:
            self.cmd.send(lineal, angular)
            self.spin(1 / HZ)
        self.parar()

    def parar(self):
        self.cmd.stop()

    def avanzar(self, metros, vel=0.15, frenar_a=0.30, timeout=30.0):
        """Avanza (o retrocede si metros < 0) usando /odom. Frena si el LiDAR ve algo a
        menos de frenar_a m en el sentido de marcha (None = sin frenado). True si llego."""
        self.esperar_sensores()
        x0, y0, _ = self.odom.pose
        signo = 1.0 if metros >= 0 else -1.0
        end = time.monotonic() + timeout
        try:
            while time.monotonic() < end:
                x, y, _ = self.odom.pose
                falta = abs(metros) - math.hypot(x - x0, y - y0)
                if falta <= 0.01:
                    return True
                if frenar_a is not None:
                    d = self.lidar.distancia(0.0 if signo > 0 else math.pi, math.radians(25))
                    if d is not None and d < frenar_a:
                        print(f"[tb4] obstaculo a {d:.2f} m: me detengo")
                        return False
                if self.bumper.activo():
                    print("[tb4] choque: me detengo")
                    return False
                v = min(vel, max(0.05, falta * 1.5))
                self.cmd.send(signo * v, 0.0)
                self.spin(1 / HZ)
            return False
        finally:
            self.parar()

    def girar(self, radianes, vel=0.8, timeout=20.0):
        """Gira en el lugar radianes (+ = izquierda) usando el yaw de /odom."""
        self.esperar_sensores()
        girado, prev = 0.0, self.odom.pose[2]
        end = time.monotonic() + timeout
        try:
            while time.monotonic() < end:
                yaw = self.odom.pose[2]
                girado += math.atan2(math.sin(yaw - prev), math.cos(yaw - prev))
                prev = yaw
                falta = abs(radianes) - abs(girado)
                if falta <= math.radians(1.5):
                    return True
                w = min(vel, max(0.25, falta * 2.0))
                self.cmd.send(0.0, math.copysign(w, radianes))
                self.spin(1 / HZ)
            return False
        finally:
            self.parar()

    def ir_a(self, x, y, vel=0.15, frenar_a=0.30):
        """Gira hacia (x, y) del marco odom y avanza en linea recta (sin evasion: para
        esquivar obstaculos usar el Piloto de proyecto_final/1_competencia/navegacion.py)."""
        self.esperar_sensores()
        px, py, yaw = self.odom.pose
        rumbo = math.atan2(y - py, x - px)
        self.girar(math.atan2(math.sin(rumbo - yaw), math.cos(rumbo - yaw)))
        return self.avanzar(math.hypot(x - px, y - py), vel, frenar_a)

    # --- salida --------------------------------------------------------------------
    def decir(self, texto, topic="/nombre_reconocido", voz=True):
        if self._anunciador is None:
            self._anunciador = Anunciador(self.node, topic, voz)
        self._anunciador.anunciar(texto)
