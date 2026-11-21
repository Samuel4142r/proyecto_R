"""Controlador Pure Pursuit para el robot Ackermann.

Lee la odometria (/odom), sigue una lista de waypoints y publica /cmd_vel.

Ley de control (modelo bicicleta, referencia en el eje trasero):
    alpha = angulo entre el rumbo del robot y el punto look-ahead
    delta = atan( 2 L sin(alpha) / Ld )
    w     = v tan(delta) / L        (el plugin AckermannSteering convierte w -> delta)

Topicos:
  sub  /odom                      nav_msgs/Odometry
  pub  /cmd_vel                   geometry_msgs/Twist
  pub  /planned_path              nav_msgs/Path        (transient local, para RViz y el logger)
  pub  /tracking/reference        geometry_msgs/PoseStamped  (punto mas cercano = pose deseada)
  pub  /tracking/lookahead        geometry_msgs/PointStamped
  pub  /tracking/cross_track_error std_msgs/Float64
  pub  /tracking/done             std_msgs/Bool
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64

from ackermann_controller import paths


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        p = self.declare_parameter
        p('path_type', 'figure8')          # figure8 | oval | custom
        p('figure8_a', 3.5)                # semieje x de la figura en 8 [m]
        p('figure8_b', 1.5)                # amplitud y de la figura en 8 [m]
        p('oval_straight', 3.0)            # largo de las rectas del ovalo [m]
        p('oval_radius', 1.2)              # radio de las curvas del ovalo [m]
        p('waypoints_x', [0.0])            # solo si path_type == custom
        p('waypoints_y', [0.0])
        p('closed_path', True)
        p('align_to_start', True)          # la ruta empieza en el robot, con su rumbo
        p('path_resolution', 0.05)         # [m]
        p('laps', 1)
        p('linear_speed', 0.5)             # [m/s]
        p('lookahead_distance', 0.6)       # Ld minimo [m]
        p('lookahead_gain', 0.0)           # Ld = Ld0 + k*v
        p('wheel_base', 0.30)              # L [m]
        p('max_steering_angle', 0.5236)    # 30 deg
        p('goal_tolerance', 0.10)          # [m]
        p('control_rate', 30.0)            # [Hz]
        p('start_delay', 2.0)              # [s] tiempo de simulacion antes de arrancar
        p('odom_topic', '/odom')
        p('wait_for_logger', False)        # esperar /data_logger/ready antes de arrancar

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.v_ref = float(gp('linear_speed'))
        self.ld0 = float(gp('lookahead_distance'))
        self.ld_k = float(gp('lookahead_gain'))
        self.L = float(gp('wheel_base'))
        self.delta_max = float(gp('max_steering_angle'))
        self.goal_tol = float(gp('goal_tolerance'))
        self.start_delay = float(gp('start_delay'))

        self.path = self._build_path()
        self.path_yaw = paths.headings(self.path)
        self.idx = 0
        self.pose = None          # (x, y, yaw, v)
        self.t_first_odom = None
        self.done = False
        self.logger_ready = not bool(gp('wait_for_logger'))

        r_min = self.L / math.tan(self.delta_max)
        k_max = paths.max_curvature(self.path)
        self.get_logger().info(
            f"Trayectoria '{gp('path_type')}': {len(self.path)} puntos, "
            f"longitud {self.path_length:.2f} m, radio minimo de la ruta {1.0 / k_max:.2f} m "
            f"(radio minimo del robot {r_min:.2f} m)")
        if 1.0 / k_max < r_min:
            self.get_logger().warn('La ruta tiene curvas mas cerradas que el radio minimo de giro!')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', latched)
        self.pub_ref = self.create_publisher(PoseStamped, '/tracking/reference', 10)
        self.pub_la = self.create_publisher(PointStamped, '/tracking/lookahead', 10)
        self.pub_cte = self.create_publisher(Float64, '/tracking/cross_track_error', 10)
        self.pub_done = self.create_publisher(Bool, '/tracking/done', latched)
        self.create_subscription(Odometry, gp('odom_topic'), self._on_odom, 20)
        self.create_subscription(Bool, '/data_logger/ready', self._on_logger_ready, latched)
        self.create_timer(1.0 / float(gp('control_rate')), self._step)
        self._publish_path()

    # ------------------------------------------------------------------ ruta
    def _build_path(self) -> np.ndarray:
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        kind = gp('path_type')
        closed = bool(gp('closed_path'))
        if kind == 'figure8':
            pts, closed = paths.figure8(gp('figure8_a'), gp('figure8_b')), True
        elif kind == 'oval':
            pts, closed = paths.oval(gp('oval_straight'), gp('oval_radius')), True
        elif kind == 'custom':
            pts = np.column_stack((gp('waypoints_x'), gp('waypoints_y'))).astype(float)
        else:
            raise ValueError(f'path_type desconocido: {kind}')
        if gp('align_to_start'):
            pts = paths.align_to_start(pts)
        pts = paths.resample(pts, float(gp('path_resolution')), closed)
        laps = max(1, int(gp('laps'))) if closed else 1
        full = np.vstack([pts] * laps)
        if closed:
            full = np.vstack((full, pts[:1]))      # cerrar la ultima vuelta
        self.path_length = float(np.sum(np.linalg.norm(np.diff(full, axis=0), axis=1)))
        return full

    def _publish_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y), yaw in zip(self.path, self.path_yaw):
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    # ------------------------------------------------------------- callbacks
    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     yaw_from_quat(q), msg.twist.twist.linear.x)
        if self.t_first_odom is None:
            self.t_first_odom = self.get_clock().now()
            self.get_logger().info(f'Odometria recibida, arranque en {self.start_delay:.1f} s')

    def _on_logger_ready(self, msg: Bool):
        if msg.data and not self.logger_ready:
            self.logger_ready = True
            self.t_first_odom = self.get_clock().now()   # el retardo cuenta desde aqui
            self.get_logger().info('Logger listo, arrancando')

    def _step(self):
        if self.pose is None or self.done or not self.logger_ready:
            return
        if (self.get_clock().now() - self.t_first_odom).nanoseconds * 1e-9 < self.start_delay:
            return
        x, y, yaw, _ = self.pose
        pos = np.array([x, y])

        # 1) punto mas cercano, buscando solo hacia adelante (evita saltos en el cruce del 8)
        window = self.path[self.idx:self.idx + 60]
        self.idx += int(np.argmin(np.linalg.norm(window - pos, axis=1)))
        ref = self.path[self.idx]
        ref_yaw = self.path_yaw[self.idx % len(self.path_yaw)]

        # error lateral con signo (izquierda positiva)
        cte = -math.sin(ref_yaw) * (x - ref[0]) + math.cos(ref_yaw) * (y - ref[1])

        # 2) fin de la trayectoria
        dist_goal = float(np.linalg.norm(self.path[-1] - pos))
        if self.idx >= len(self.path) - 3 and dist_goal < max(self.goal_tol, 0.3):
            self._finish()
            return

        # 3) punto look-ahead
        v = self.v_ref
        ld = self.ld0 + self.ld_k * v
        j = self.idx
        while j < len(self.path) - 1 and np.linalg.norm(self.path[j] - pos) < ld:
            j += 1
        target = self.path[j]

        # 4) ley de Pure Pursuit
        dx, dy = target[0] - x, target[1] - y
        alpha = math.atan2(dy, dx) - yaw
        ld_real = max(math.hypot(dx, dy), 1e-3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), ld_real)
        delta = max(-self.delta_max, min(self.delta_max, delta))

        # bajar la velocidad al acercarse a la meta
        remaining = (len(self.path) - 1 - self.idx) * float(self.get_parameter('path_resolution').value)
        v = min(v, max(0.12, 0.8 * remaining))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v * math.tan(delta) / self.L
        self.pub_cmd.publish(cmd)

        now = self.get_clock().now().to_msg()
        r = PoseStamped()
        r.header.stamp, r.header.frame_id = now, 'odom'
        r.pose.position.x, r.pose.position.y = float(ref[0]), float(ref[1])
        r.pose.orientation.z, r.pose.orientation.w = math.sin(ref_yaw / 2), math.cos(ref_yaw / 2)
        self.pub_ref.publish(r)
        la = PointStamped()
        la.header.stamp, la.header.frame_id = now, 'odom'
        la.point.x, la.point.y = float(target[0]), float(target[1])
        self.pub_la.publish(la)
        self.pub_cte.publish(Float64(data=float(cte)))

    def _finish(self):
        self.done = True
        self.pub_cmd.publish(Twist())
        self.pub_done.publish(Bool(data=True))
        self.get_logger().info('Trayectoria completada. Robot detenido.')

    def stop(self):
        self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
