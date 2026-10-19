"""관절 서보 능동 제어 (3마디용): 각 마디가 자기 위치의 배관 방향을 향하도록 관절 각도 명령.

follow_mode
  oracle — 시뮬이 아는 배관 중심선 + 정답 위치 사용. "센서가 완벽할 때"의 상한 확인용.
  sensor — 실물과 같은 정보만 사용:
           앞마디 목표 방향 = 전방 8x8 ToF 에서 가장 트인 방향 (광선 방향을 거리로 가중 평균)
           가운데·뒤 마디 목표 방향 = 앞마디가 같은 위치(엔코더 거리)를 지날 때의 자세(IMU 적분, /odom)
                                     를 기억해 두었다가 따라감 (follow-the-leader)

관절 구조(three_segment): 앞마디 --[피치(y) -> 요(z)]-- 가운데 --[피치 -> 요]-- 뒤
뒤 마디의 x축(진행 방향)을 앞 마디 좌표로 나타낸 d 에 대해  요 = asin(d_y), 피치 = atan2(-d_z, d_x)

sub  /ground_truth/odom (앞마디 정답 자세), /joint_states (실제 관절각)
pub  /servo/<joint> (std_msgs/Float64, rad) -> 브리지 -> Gazebo JointPositionController
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float64, Int64MultiArray

from pipe_robot_sim.generate_world import default_courses_file, overrides
from pipe_robot_sim.odometry import q_from_msg
from pipe_robot_sim.pipe_geometry import build_course, load_config, nearest_tangent


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                     [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                     [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def align(R, t):
    """R의 x축을 t로 돌리는 최소 회전을 적용한 자세."""
    v = R[:, 0]
    axis = np.cross(v, t)
    s, c = np.linalg.norm(axis), float(np.dot(v, t))
    if s < 1e-9:
        return R
    k = axis / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    ang = math.atan2(s, c)
    return (np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)) @ R


class JointFollow(Node):
    def __init__(self):
        super().__init__('joint_follow')
        d = self.declare_parameter
        self.joints = list(d('servo_joints', ['']).value)          # [앞 피치, 앞 요, 뒤 피치, 뒤 요]
        self.modes = list(d('servo_modes', ['']).value)
        self.L = list(d('seg_len', [0.09, 0.1, 0.11]).value)
        self.gap = d('joint_gap', 0.035).value
        self.lim = math.radians(d('joint_limit_deg', 60.0).value)
        course = d('course', 'straight').value
        path = d('courses_file', '').value or default_courses_file()
        self.course = build_course(load_config(path), course,
                                   overrides(d('fitting90', '').value, d('fitting45', '').value))

        self.mode = d('follow_mode', 'oracle').value
        self.R0 = None
        self.p0 = None
        self.q = {}
        # sensor 모드 상태
        self.wr = d('wheel_radius', 0.012).value
        self.cpr = d('encoder_cpr', 2800).value
        self.gain = d('tof_gain', 1.5).value
        self.Rest = None                  # 앞마디 자세 추정 (/odom, IMU 적분)
        self.s = 0.0                      # 엔코더 이동거리
        self.enc0 = None
        self.hist = []                    # (s, R) 앞마디 자세 기록
        self.free_dir = np.array([1.0, 0.0, 0.0])   # 앞마디 좌표, 전방 ToF 로 본 가장 트인 방향
        self.last_dirs = None
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 10)
        if self.mode == 'sensor':
            self.create_subscription(Odometry, 'odom', self.on_odom, 10)
            self.create_subscription(Int64MultiArray, 'wheel_encoders', self.on_enc, 10)
            self.create_subscription(PointCloud2, 'tof/front/points', self.on_tof, 10)
        self.create_subscription(JointState, 'joint_states',
                                 lambda m: self.q.update(dict(zip(m.name, m.position))), 10)
        self.pubs = {j: self.create_publisher(Float64, f'servo/{j}', 10)
                     for j, m in zip(self.joints, self.modes) if m == 'active'}
        self.create_timer(0.02, self.step)
        self.get_logger().info(f'능동 관절: {list(self.pubs)} (mode={self.mode})')

    # ---- sensor 모드 입력 ----
    def on_odom(self, msg):
        self.Rest = quat_to_rot(q_from_msg(msg.pose.pose.orientation))

    def on_enc(self, msg):
        c = np.array(msg.data, dtype=float)
        if self.enc0 is None:
            self.enc0 = c
        self.s = float(np.mean(c - self.enc0)) * 2 * math.pi / self.cpr * self.wr
        if self.Rest is not None and (not self.hist or self.s - self.hist[-1][0] > 0.002):
            self.hist.append((self.s, self.Rest.copy()))

    def on_tof(self, msg):
        pts = np.array([[p[0], p[1], p[2]] for p in
                        pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)], dtype=float)
        r = np.linalg.norm(pts, axis=1)
        ok = np.isfinite(r) & (r > 1e-4)
        dirs = np.zeros_like(pts)
        dirs[ok] = pts[ok] / r[ok, None]
        if self.last_dirs is not None:          # 최대 거리 넘는 광선: 방향은 지난 값, 거리는 최대
            dirs[~ok] = self.last_dirs[~ok]
        else:
            dirs[~ok] = [1.0, 0.0, 0.0]
        self.last_dirs = dirs
        rmax = 2.0
        rr = np.where(ok, np.minimum(r, rmax), rmax)
        w = (rr / rmax) ** 3
        d = (dirs * w[:, None]).sum(axis=0)
        d /= np.linalg.norm(d)
        # 정면에서 벗어난 각도를 gain 배 (센서 시야가 좁아 실제 꺾임보다 작게 나옴)
        ang = math.acos(max(-1.0, min(1.0, d[0])))
        if ang > 1e-6:
            axis = np.cross([1.0, 0.0, 0.0], d)
            axis /= np.linalg.norm(axis)
            ang = min(ang * self.gain, math.radians(70))
            ex = np.array([1.0, 0.0, 0.0])
            d = math.cos(ang) * ex + math.sin(ang) * np.cross(axis, ex)   # 정면을 트인 쪽으로 ang 만큼
        self.free_dir = 0.7 * self.free_dir + 0.3 * d
        self.free_dir /= np.linalg.norm(self.free_dir)

    def hist_R(self, back):
        """앞마디가 지금보다 back [m] 뒤에 있을 때의 자세 (기록 없으면 가장 오래된 것)"""
        target = self.s - back
        if not self.hist or target <= self.hist[0][0]:
            return self.hist[0][1] if self.hist else self.Rest
        lo, hi = 0, len(self.hist) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.hist[mid][0] < target:
                lo = mid + 1
            else:
                hi = mid
        return self.hist[lo][1]

    def on_gt(self, msg):
        pos = msg.pose.pose.position
        self.p0 = np.array([pos.x, pos.y, pos.z])
        self.R0 = quat_to_rot(q_from_msg(msg.pose.pose.orientation))

    def _angles(self, d):
        d = d / np.linalg.norm(d)
        yaw = math.asin(max(-1.0, min(1.0, d[1])))
        pitch = math.atan2(-d[2], d[0])
        c = lambda a: max(-self.lim, min(self.lim, a))  # noqa: E731
        return c(pitch), c(yaw)

    def step(self):
        if not all(j in self.q for j in self.joints):
            return
        if self.mode == 'sensor':
            return self.step_sensor()
        if self.R0 is None:
            return
        L0, L1, L2 = self.L
        g = self.gap
        qp1, qy1 = self.q[self.joints[0]], self.q[self.joints[1]]
        qp2, qy2 = (self.q[self.joints[2]], self.q[self.joints[3]]) if len(self.joints) >= 4 else (0.0, 0.0)
        # 실제 관절각으로 각 마디 위치·자세 계산 (정기구학)
        R1 = self.R0 @ ry(qp1) @ rz(qy1)
        j1 = self.p0 + self.R0 @ np.array([-(L0 / 2 + g / 2), 0, 0])
        p1 = j1 + R1 @ np.array([-(g / 2 + L1 / 2), 0, 0])
        j2 = j1 + R1 @ np.array([-(g + L1), 0, 0])
        R2 = R1 @ ry(qp2) @ rz(qy2)
        p2 = j2 + R2 @ np.array([-(g / 2 + L2 / 2), 0, 0])

        # 목표: 각 마디가 자기 위치의 배관 방향을 향함 (앞마디는 앞쪽 끝 기준으로 미리 봄)
        t0 = nearest_tangent(self.course, self.p0 + self.R0 @ np.array([L0 / 2, 0, 0]))
        t1 = nearest_tangent(self.course, p1)
        t2 = nearest_tangent(self.course, p2)
        R0d = align(self.R0, t0)          # 앞마디 목표 자세 (롤은 현재 그대로)
        R1d = align(R1, t1)
        cmd = {}
        cmd[self.joints[0]], cmd[self.joints[1]] = self._angles(R0d.T @ t1)
        if len(self.joints) >= 4:
            cmd[self.joints[2]], cmd[self.joints[3]] = self._angles(R1d.T @ t2)
        for j, pub in self.pubs.items():
            pub.publish(Float64(data=float(cmd[j])))


    def step_sensor(self):
        if self.Rest is None or not self.hist:
            return
        L0, L1, L2 = self.L
        g = self.gap
        d1 = L0 / 2 + g + L1 / 2                 # 앞마디 중심 ~ 가운데 마디 중심 (경로 거리)
        d2 = d1 + L1 / 2 + g + L2 / 2
        Rn = self.Rest
        ex = np.array([1.0, 0.0, 0.0])
        t1 = Rn.T @ (self.hist_R(d1) @ ex)       # 가운데 마디가 향해야 할 방향 (앞마디 좌표)
        t2 = Rn.T @ (self.hist_R(d2) @ ex)
        qp1, qy1 = self.q[self.joints[0]], self.q[self.joints[1]]
        R1 = ry(qp1) @ rz(qy1)                   # 가운데 마디 실제 자세 (앞마디 좌표)
        R0d = align(np.eye(3), self.free_dir)    # 앞마디 목표: ToF 로 본 트인 방향
        R1d = align(R1, t1)
        cmd = {}
        cmd[self.joints[0]], cmd[self.joints[1]] = self._angles(R0d.T @ t1)
        if len(self.joints) >= 4:         # 3마디일 때만 뒤 관절
            cmd[self.joints[2]], cmd[self.joints[3]] = self._angles(R1d.T @ t2)
        for j, pub in self.pubs.items():
            pub.publish(Float64(data=float(cmd[j])))
        fd = self.free_dir
        self.get_logger().info(
            f's={self.s:.3f} 트인방향(피치,요)=({math.degrees(math.atan2(-fd[2], fd[0])):.0f},'
            f'{math.degrees(math.atan2(fd[1], fd[0])):.0f}) 명령={[round(math.degrees(v)) for v in cmd.values()]} '
            f'실제={[round(math.degrees(self.q[j])) for j in self.joints]}', throttle_duration_sec=0.5)


def main():
    rclpy.init()
    rclpy.spin(JointFollow())


if __name__ == '__main__':
    main()
