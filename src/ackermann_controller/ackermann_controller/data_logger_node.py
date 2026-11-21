"""Nodo de registro: guarda en CSV el tiempo, la pose deseada, la odometria y la pose real.

Archivos generados en `output_dir`:
  <run>.csv       t, pose deseada, odometria, ground truth, comandos y error lateral
  <run>_path.csv  la trayectoria planificada completa (x, y)

Empieza a grabar cuando el controlador publica la primera referencia y cierra el
archivo al recibir /tracking/done (o al detener el nodo con Ctrl+C).
"""
import csv
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64

HEADER = ['t', 'x_ref', 'y_ref', 'yaw_ref',
          'x_odom', 'y_odom', 'yaw_odom',
          'x_gt', 'y_gt', 'yaw_gt',
          'v_gt', 'v_cmd', 'w_cmd', 'cte_odom']


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DataLogger(Node):

    def __init__(self):
        super().__init__('data_logger')
        self.declare_parameter('output_dir', os.path.join(os.getcwd(), 'results'))
        self.declare_parameter('run_name', '')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('shutdown_on_done', True)

        out_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        os.makedirs(out_dir, exist_ok=True)
        name = self.get_parameter('run_name').value or datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(out_dir, f'{name}.csv')
        self.path_csv = os.path.join(out_dir, f'{name}_path.csv')

        self.ref = self.odom = self.gt = None
        self.cmd = Twist()
        self.cte = float('nan')
        self.t0 = None
        self.rows = 0
        self.closed = False
        self.f = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseStamped, '/tracking/reference', self._on_ref, 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 20)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', m), 20)
        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_subscription(Float64, '/tracking/cross_track_error',
                                 lambda m: setattr(self, 'cte', m.data), 10)
        self.create_subscription(Path, '/planned_path', self._on_path, latched)
        self.create_subscription(Bool, '/tracking/done', self._on_done, latched)
        # Avisa al controlador cuando ya recibe odometria y ground truth
        self.pub_ready = self.create_publisher(Bool, '/data_logger/ready', latched)
        self.ready_sent = False
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self._write)
        self.get_logger().info(f'Registrando en {self.csv_path}')

    def _on_ref(self, msg):
        self.ref = msg
        if self.f is None:
            self.f = open(self.csv_path, 'w', newline='')
            self.w = csv.writer(self.f)
            self.w.writerow(HEADER)

    def _on_path(self, msg: Path):
        with open(self.path_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y'])
            for ps in msg.poses:
                w.writerow([f'{ps.pose.position.x:.4f}', f'{ps.pose.position.y:.4f}'])

    def _write(self):
        if not self.ready_sent and self.odom is not None and self.gt is not None:
            self.pub_ready.publish(Bool(data=True))
            self.ready_sent = True
            self.get_logger().info('Logger listo (odom + ground truth recibidos)')
        if self.closed or self.f is None or self.odom is None or self.gt is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        r, o, g = self.ref.pose, self.odom.pose.pose, self.gt.pose.pose
        self.w.writerow([
            f'{now - self.t0:.3f}',
            f'{r.position.x:.4f}', f'{r.position.y:.4f}', f'{yaw_of(r.orientation):.4f}',
            f'{o.position.x:.4f}', f'{o.position.y:.4f}', f'{yaw_of(o.orientation):.4f}',
            f'{g.position.x:.4f}', f'{g.position.y:.4f}', f'{yaw_of(g.orientation):.4f}',
            f'{self.gt.twist.twist.linear.x:.4f}',
            f'{self.cmd.linear.x:.4f}', f'{self.cmd.angular.z:.4f}', f'{self.cte:.4f}'])
        self.rows += 1

    def _on_done(self, msg: Bool):
        if msg.data:
            self.close()
            if self.get_parameter('shutdown_on_done').value:
                raise SystemExit

    def close(self):
        if not self.closed and self.f is not None:
            self.f.close()
            self.get_logger().info(f'CSV guardado: {self.csv_path} ({self.rows} filas)')
        self.closed = True


def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
