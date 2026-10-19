"""기본 위치추정(추측항법) + 정답 비교. 센서퓨전 실험의 출발점(베이스라인).

추정: 바퀴 엔코더 평균 이동거리 ds 를 IMU 자세로 회전해 누적  p += R(q) [ds, 0, 0]
  imu_mode: gyro        -> 자이로 각속도를 적분해 자세 계산 (실제 MPU6050과 같은 조건, 드리프트 있음)
            orientation -> Gazebo IMU가 주는 자세를 그대로 사용 (자세 오차 없는 기준선)
정답: Gazebo OdometryPublisher (/ground_truth/odom)

sub  /wheel_encoders, /imu, /ground_truth/odom
pub  /odom (nav_msgs/Odometry, 추정), /odom_path, /ground_truth/path (nav_msgs/Path)
TF   world -> base_link (정답, RViz에서 로봇 모델 표시용), world -> base_link_est (추정)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster


def q_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])


def q_rot(q, v):
    w, x, y, z = q
    r = np.array([[1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                  [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                  [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])
    return r @ v


def q_from_msg(o):
    return np.array([o.w, o.x, o.y, o.z])


class Odom(Node):
    def __init__(self):
        super().__init__('odometry')
        d = self.declare_parameter
        self.joints = list(d('wheel_joints', ['']).value)
        self.r = d('wheel_radius', 0.012).value
        self.cpr = d('encoder_cpr', 2800).value
        self.imu_mode = d('imu_mode', 'gyro').value

        self.p = None              # 추정 위치 (정답 첫 값으로 초기화)
        self.q = None              # 추정 자세 [w x y z]
        self.prev_counts = None
        self.prev_imu_t = None
        self.dist_est = 0.0
        self.dist_true = 0.0
        self.prev_true = None
        self.true_p = None

        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.path_pub = self.create_publisher(Path, 'odom_path', 10)
        self.gt_path_pub = self.create_publisher(Path, 'ground_truth/path', 10)
        self.path = Path()
        self.gt_path = Path()
        self.path.header.frame_id = self.gt_path.header.frame_id = 'world'

        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 50)
        self.create_subscription(Int64MultiArray, 'wheel_encoders', self.on_enc, 50)
        self.create_timer(0.1, self.publish_paths)
        self.create_timer(1.0, self.report)

    # ---- 정답 ----
    def on_gt(self, msg):
        pose = msg.pose.pose
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        if self.p is None:
            self.p = p.copy()
            self.q = q_from_msg(pose.orientation)
        if self.prev_true is not None:
            self.dist_true += float(np.linalg.norm(p - self.prev_true))
        self.prev_true = p
        self.true_p = p
        self._send_tf('base_link', msg.header.stamp, p, q_from_msg(pose.orientation))
        if not self.gt_path.poses or np.linalg.norm(p - self._last(self.gt_path)) > 0.005:
            self.gt_path.poses.append(self._pose(msg.header.stamp, p, q_from_msg(pose.orientation)))

    # ---- IMU ----
    def on_imu(self, msg):
        if self.q is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.imu_mode == 'orientation':
            self.q = q_from_msg(msg.orientation)
        elif self.prev_imu_t is not None:
            dt = t - self.prev_imu_t
            w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            ang = np.linalg.norm(w) * dt
            if ang > 1e-12:
                axis = w / np.linalg.norm(w)
                dq = np.concatenate([[math.cos(ang / 2)], axis * math.sin(ang / 2)])
                self.q = q_mul(self.q, dq)          # 몸체 좌표 각속도 -> 오른쪽 곱
                self.q /= np.linalg.norm(self.q)
        self.prev_imu_t = t

    # ---- 엔코더 ----
    def on_enc(self, msg):
        counts = np.array(msg.data, dtype=float)
        if self.prev_counts is None or self.p is None:
            self.prev_counts = counts
            return
        ds = float(np.mean(counts - self.prev_counts)) * 2 * math.pi / self.cpr * self.r
        self.prev_counts = counts
        self.dist_est += ds
        self.p = self.p + q_rot(self.q, np.array([ds, 0.0, 0.0]))

        stamp = self.get_clock().now().to_msg()
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'world'
        od.child_frame_id = 'base_link_est'
        od.pose.pose = self._pose(stamp, self.p, self.q).pose
        self.odom_pub.publish(od)
        self._send_tf('base_link_est', stamp, self.p, self.q)
        if not self.path.poses or np.linalg.norm(self.p - self._last(self.path)) > 0.005:
            self.path.poses.append(self._pose(stamp, self.p, self.q))

    # ---- 출력 ----
    def publish_paths(self):
        now = self.get_clock().now().to_msg()
        self.path.header.stamp = self.gt_path.header.stamp = now
        self.path_pub.publish(self.path)
        self.gt_path_pub.publish(self.gt_path)

    def report(self):
        if self.true_p is None or self.p is None:
            return
        err = float(np.linalg.norm(self.p - self.true_p))
        self.get_logger().info(
            f'이동거리 정답 {self.dist_true:.3f} m / 엔코더 {self.dist_est:.3f} m | 위치 오차 {err * 1000:.1f} mm')

    def _send_tf(self, child, stamp, p, q):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
        t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = map(float, q)
        self.tf.sendTransform(t)

    @staticmethod
    def _pose(stamp, p, q):
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = 'world'
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = map(float, q)
        return ps

    @staticmethod
    def _last(path):
        p = path.poses[-1].pose.position
        return np.array([p.x, p.y, p.z])


def main():
    rclpy.init()
    rclpy.spin(Odom())


if __name__ == '__main__':
    main()
