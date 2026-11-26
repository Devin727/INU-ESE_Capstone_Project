"""wp6 (벽 밀착 6륜 2모듈) 가운데 관절 제어.

가운데 유니버설 관절(피치 -> 요)을 서보로 굽혀 앞 모듈을 엘보 쪽으로 미리 돌림 (R61 엘보는 수동 관절로는 못 지나감, docs/05).
  ctrl_mode oracle: 배관 모양과 정답 자세를 앎 (실물은 카메라 등으로 엘보 방향을 판단해야 함)
    모듈마다 바퀴 줄이 한 줄이라, 바퀴 줄 면이 그 자리 배관 축에 수직이어야 끼지 않음 (미리 크게 꺾으면 직관에서 대각선으로 끼임)
    앞 모듈 목표 방향 = 앞 바퀴 줄에서 lookahead 앞 지점의 배관 방향
      아래로 꺾이는 엘보만 lookahead_down 을 씀: 미리 굽혀도 무게가 앞 모듈을 수직관으로 끌어 줌.
      수평·위 엘보는 미리 굽히면 바퀴 줄이 직관 안에서 대각선으로 끼임 (2026-10-31 시험)
    뒤 모듈 목표 방향 = 뒤 바퀴 줄 자리의 배관 방향 (rear_ref path) 또는 지금 뒤 모듈 방향 (rear_ref actual)
      actual: 모듈마다 바퀴가 한 줄이라 굽힘 반작용에 뒤 모듈이 기울어도 앞 모듈의 절대 방향을 맞춤
    -> 두 방향 사이 회전을 관절각(피치, 요)으로 바꿔 /servo/mid_pitch_joint, /servo/mid_yaw_joint 로 보냄
  ctrl_mode straight: 관절 0 (일자 유지)
바퀴는 motor_driver (/cmd_vel).

sub /gz_poses
pub /servo/mid_pitch_joint, /servo/mid_yaw_joint, /wp_ctrl/state
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

from pipe_robot_sim.generate_world import default_courses_file, overrides
from pipe_robot_sim.odometry import quat_to_rot
from pipe_robot_sim.pipe_geometry import build_course, load_config

ROBOT = 'pipe_robot'


class WpCtrl(Node):
    def __init__(self):
        super().__init__('wp_ctrl')
        d = self.declare_parameter
        self.mode = d('ctrl_mode', 'oracle').value
        self.ring_f = np.array([d('ring_front_x', 0.025).value, 0.0, 0.0])   # base_link 좌표의 앞 바퀴 줄 중심
        self.ring_r = np.array([d('ring_rear_x', -0.08).value, 0.0, 0.0])    # rear_body 좌표의 뒤 바퀴 줄 중심
        self.lookahead = d('lookahead', 0.0).value                            # [m]
        self.lookahead_down = d('lookahead_down', 0.02).value                 # [m]
        self.rear_ref = d('rear_ref', 'path').value                           # path | actual
        self.lim = d('mid_cmd_limit', 1.4).value                              # [rad]
        path = d('courses_file', '').value or default_courses_file()
        cfg = load_config(path)
        b = build_course(cfg, d('course', 'straight').value,
                         overrides(d('fitting90', '').value, d('fitting45', '').value))
        self.pts = np.array([p[0] for p in b.path])
        self.tang = np.array([p[1] for p in b.path])
        self.seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(self.pts, axis=0), axis=1))]
        self.poses = {}
        self.state = ''
        self.create_subscription(TFMessage, 'gz_poses', self.on_poses, 10)
        self.pubs = {j: self.create_publisher(Float64, f'servo/{j}', 10) for j in ('mid_pitch_joint', 'mid_yaw_joint')}
        self.state_pub = self.create_publisher(String, 'wp_ctrl/state', 10)
        self.create_timer(0.02, self.step)

    def on_poses(self, msg):
        for t in msg.transforms:
            tr, r = t.transform.translation, t.transform.rotation
            self.poses[t.child_frame_id] = (np.array([tr.x, tr.y, tr.z]), quat_to_rot((r.w, r.x, r.y, r.z)))

    def world(self, link):
        pm, Rm = self.poses[ROBOT]
        pl, Rl = self.poses[link]
        return pm + Rm @ pl, Rm @ Rl

    def tangent_at(self, s):
        i = int(np.clip(np.searchsorted(self.seg, s), 0, len(self.seg) - 1))
        return self.tang[i]

    def send(self, pitch, yaw):
        self.pubs['mid_pitch_joint'].publish(Float64(data=float(np.clip(pitch, -self.lim, self.lim))))
        self.pubs['mid_yaw_joint'].publish(Float64(data=float(np.clip(yaw, -self.lim, self.lim))))

    def s_of(self, p):
        return self.seg[int(np.argmin(np.linalg.norm(self.pts - p, axis=1)))]

    def step(self):
        if self.mode != 'oracle':
            self.send(0.0, 0.0)
            return
        if not all(k in self.poses for k in (ROBOT, 'base_link', 'rear_body')):
            return
        pf, Rf = self.world('base_link')
        pr, Rr = self.world('rear_body')
        s = self.s_of(pf + Rf @ self.ring_f)
        down = self.tangent_at(s + 0.05)[2] < -0.1                           # 앞쪽 배관이 아래로 꺾임
        want = self.tangent_at(s + (self.lookahead_down if down else self.lookahead))
        rear_want = self.tangent_at(self.s_of(pr + Rr @ self.ring_r)) if self.rear_ref == 'path' else Rr[:, 0]
        # 앞 모듈을 지금 자세에서 최소 회전으로 want 에 맞춘 자세 (축 둘레 굴림은 그대로)
        xf = Rf[:, 0]
        ax = np.cross(xf, want)
        sn, cs = np.linalg.norm(ax), float(np.dot(xf, want))
        if sn > 1e-9:
            k = ax / sn
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            Rf = (np.eye(3) + sn * K + (1 - cs) * K @ K) @ Rf
        # 뒤 모듈 축을 그 앞 모듈 좌표로: v = Ry(p) Rz(y) x = (cos p cos y, sin y, -sin p cos y)
        v = Rf.T @ rear_want
        yaw = math.asin(float(np.clip(v[1], -1, 1)))
        pitch = math.atan2(-float(v[2]), float(v[0]))
        self.send(pitch, yaw)
        state = 'BEND' if abs(pitch) + abs(yaw) > 0.1 else 'STRAIGHT'
        if state != self.state:
            self.state = state
            self.get_logger().info(f'-> {state} (앞 바퀴 줄 s={s:.2f} m, 피치 {math.degrees(pitch):+.0f}°, 요 {math.degrees(yaw):+.0f}°)')
        self.state_pub.publish(String(data=state))


def main():
    rclpy.init()
    rclpy.spin(WpCtrl())


if __name__ == '__main__':
    main()
