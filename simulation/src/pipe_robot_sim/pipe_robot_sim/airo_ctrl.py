"""AIRo-7.1 (airo71) 제어.

논문 방식 (arXiv 2411.03660): 가운데 관절은 위치 제어 + PWM 토크 제한으로 계속 접혀 벽을 누르고, 앞뒤 관절은 토션 스프링.
엘보는 관절이 수동으로 따라 굽으며 지나감. 굴림 바퀴(양 끝 반구형)로 로봇을 배관 축 둘레로 돌려 굽힘 평면을 맞춤 (논문은 조종자 명령).

이 노드
  - j2: /servo/j2_joint 에 j2_target (벽 너머 각) 을 계속 보냄 -> xacro 토크 한계(1.32 N·m)로 누름
  - 굴림: 굽힘 축(base_link y)을 원하는 방향에 맞추는 PD 토크를 양 끝 굴림 바퀴에 줌
      직관: 굽힘 평면 수직 (지그재그가 위아래 벽을 누름, 바퀴가 바닥 먼지를 피하진 않지만 중력에 안정)
      ctrl_mode oracle: 머리가 엘보 곡선 시작 align_before 앞에 오면 머리 끝 바퀴를 엘보 안쪽 벽으로 (굽힘 평면 = 엘보 평면),
      꼬리가 엘보를 빠져나가면 다시 직관 자세
      (배관 모양·정답 자세를 씀 -> 실물은 조종자나 IMU+전방 센서로 같은 판단을 해야 함)
  - 주행은 motor_driver (/cmd_vel)

sub /gz_poses, /joint_states
pub /servo/j2_joint, /motor/front_roll_joint/torque, /motor/rear_roll_joint/torque, /airo_ctrl/state, /airo_ctrl/roll_err (deg)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

from pipe_robot_sim.generate_world import default_courses_file, overrides
from pipe_robot_sim.odometry import quat_to_rot
from pipe_robot_sim.pipe_geometry import build_course, load_config

ROBOT = 'pipe_robot'


class AiroCtrl(Node):
    def __init__(self):
        super().__init__('airo_ctrl')
        d = self.declare_parameter
        self.mode = d('ctrl_mode', 'oracle').value
        self.j2_target = d('j2_target', 2.0).value
        self.j13_target = d('j13_target', 0.0).value
        # j2 토크 일정 (j2_sched: true). 위치 제어기(P = j2_p, 토크 한계 = xacro j2_torque)에 '현재각 + 토크/P' 를 보내 원하는 토크를 냄
        #   직관 j2_tau_straight, 머리가 엘보에 들어갈 때(곡선 시작 enter_before 앞 ~ enter_len) j2_tau_enter (약하게: 쐐기 방지),
        #   그 뒤 꼬리가 나올 때까지 j2_tau_pass (세게: 머리가 꺾여 매달리면 지그재그가 펴져 바퀴가 뜸)
        self.j2_sched = d('j2_sched', False).value
        self.j2_p = d('j2_p', 20.0).value
        self.tau_straight = d('j2_tau_straight', 1.32).value
        self.tau_enter = d('j2_tau_enter', 0.5).value
        self.tau_pass = d('j2_tau_pass', 2.55).value
        self.enter_before = d('enter_before', 0.05).value
        self.enter_len = d('enter_len', 0.15).value
        self.j2 = None   # 0 이 아니면 j1, j3 도 서보로 이 각도를 보냄 (비교 시험용)
        self.kp = d('roll_kp', 3.0).value
        self.kd = d('roll_kd', 0.3).value
        self.tmax = d('roll_max_torque', 2.3).value
        self.sign = d('roll_sign', 1.0).value
        self.align_pre = d('align_before', 0.30).value
        self.roll_test = math.radians(d('roll_test_deg', 0.0).value)   # 시험용: 직관 목표 자세를 이만큼 굴림
        self.head_off = d('head_offset', 0.0925).value     # base_link 원점 -> 앞 끝 [m] (Le/2 + re)
        self.tail_off = d('tail_offset', -0.4175).value    # base_link 원점 -> 뒤 끝 [m] (-(Le/2 + 2 Lm + Le + re))
        course = d('course', 'straight').value
        path = d('courses_file', '').value or default_courses_file()
        cfg = load_config(path)
        fit = overrides(d('fitting90', '').value, d('fitting45', '').value)
        b = build_course(cfg, course, fit)
        self.pts = np.array([p[0] for p in b.path])
        self.tang = np.array([p[1] for p in b.path])
        self.ups = np.array([p[2] for p in b.path])
        self.seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(self.pts, axis=0), axis=1))]
        self.elbows = []
        for name, s0 in b.features:
            if name == 'cap':
                continue
            base, direction = name.split(':')
            f = cfg['fittings'][fit.get(base, base)]
            ang = math.radians(f['angle'])
            leg = max(f['Z'] - f['R'] * math.tan(ang / 2), 0.0)
            i = self.idx(s0 + leg)
            t, u = self.tang[i], self.ups[i]
            left = np.cross(u, t)
            axis = {'left': u, 'right': -u, 'up': -left, 'down': left}[direction]
            self.elbows.append(dict(name=name, s_bend=s0 + leg, s1=s0 + 2 * leg + f['R'] * ang, axis=axis))
        self.get_logger().info('엘보: ' + ', '.join(f"{e['name']} {e['s_bend']:.2f}~{e['s1']:.2f} m" for e in self.elbows))
        self.poses = {}
        self.prev = None
        self.state = ''
        self.create_subscription(TFMessage, 'gz_poses', self.on_poses, 10)
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.j2_pub = self.create_publisher(Float64, 'servo/j2_joint', 10)
        self.j13_pubs = [self.create_publisher(Float64, f'servo/{j}_joint', 10) for j in ('j1', 'j3')]
        self.roll_pubs = [self.create_publisher(Float64, f'motor/{j}_roll_joint/torque', 10) for j in ('front', 'rear')]
        self.state_pub = self.create_publisher(String, 'airo_ctrl/state', 10)
        self.err_pub = self.create_publisher(Float64, 'airo_ctrl/roll_err', 10)
        self.create_timer(0.02, self.step)

    def idx(self, s):
        return int(np.clip(np.searchsorted(self.seg, s), 0, len(self.seg) - 1))

    def s_of(self, p):
        return int(np.argmin(np.linalg.norm(self.pts - p, axis=1)))

    def on_poses(self, msg):
        for t in msg.transforms:
            tr, r = t.transform.translation, t.transform.rotation
            self.poses[t.child_frame_id] = (np.array([tr.x, tr.y, tr.z]), quat_to_rot((r.w, r.x, r.y, r.z)))

    def on_js(self, msg):
        if 'j2_joint' in msg.name:
            self.j2 = msg.position[msg.name.index('j2_joint')]

    def base(self):
        pm, Rm = self.poses[ROBOT]
        pl, Rl = self.poses['base_link']
        return pm + Rm @ pl, Rm @ Rl

    def step(self):
        if not self.j2_sched:
            self.j2_pub.publish(Float64(data=float(self.j2_target)))
        if self.j13_target:
            for p in self.j13_pubs:
                p.publish(Float64(data=float(self.j13_target)))
        if ROBOT not in self.poses or 'base_link' not in self.poses:
            return
        p, R = self.base()
        x, y = R[:, 0], R[:, 1]
        i = self.s_of(p)
        t = self.tang[i]
        s_head = self.seg[self.s_of(p + self.head_off * x)]
        s_tail = self.seg[self.s_of(p + self.tail_off * x)]
        elbow = None
        if self.mode == 'oracle':
            for e in self.elbows:
                if s_head > e['s_bend'] - self.align_pre and s_tail < e['s1'] + 0.05:
                    elbow = e
                    break
        if elbow is None:
            # 직관: 굽힘 축(base_link y)을 수평으로 = 지그재그가 위아래 벽을 누름. 앞뒤 대칭이라 가까운 쪽 (최대 90°)
            state = 'STRAIGHT'
            want = np.cross(self.ups[i], t)
            want = want - np.dot(want, t) * t
            want /= np.linalg.norm(want)
            if self.roll_test:
                c, sn = math.cos(self.roll_test), math.sin(self.roll_test)
                want = c * want + sn * np.cross(t, want)
            if np.dot(want, y) < 0:
                want = -want
            cur = y - np.dot(y, t) * t
        else:
            # 엘보: 머리 끝 바퀴가 엘보 안쪽 벽에 닿게 (최대 180° 굴림).
            # 머리가 바깥 벽을 타면 휘어 오는 벽에 정면으로 박혀 끼임 (Oka 2022 박사논문: 끼이면 후진해 앞바퀴를 안쪽 벽에 대고 재전진)
            state = f"ELBOW {elbow['name']}"
            want = np.cross(elbow['axis'], t)          # 엘보 안쪽 방향 (굽힘 축 x 진행 방향)
            want = want - np.dot(want, t) * t
            want /= np.linalg.norm(want)
            ph = p + self.head_off * x
            cur = ph - self.pts[self.s_of(ph)]          # 배관 중심 -> 머리 끝 (머리가 닿는 쪽)
            cur = cur - np.dot(cur, t) * t
        if self.j2_sched and self.j2 is not None:
            tau, phase = self.tau_straight, ''
            for e in self.elbows:
                if e['s_bend'] - self.enter_before < s_head < e['s_bend'] + self.enter_len:
                    tau, phase = self.tau_enter, ' 진입'
                    break
                if s_head >= e['s_bend'] + self.enter_len and s_tail < e['s1'] + 0.05:
                    tau, phase = self.tau_pass, ' 통과'
                    break
            state += phase
            self.j2_pub.publish(Float64(data=float(self.j2 + tau / self.j2_p)))
        err = math.atan2(float(np.dot(np.cross(cur, want), t)), float(np.dot(cur, want)))
        if self.prev is not None and state == self.state:   # ±180° 근처에서 방향이 매번 뒤집히지 않게 직전 오차에 이어 붙임
            err += 2 * math.pi * round((self.prev[1] - err) / (2 * math.pi))
        now = self.get_clock().now().nanoseconds * 1e-9
        rate = 0.0
        if self.prev is not None and now > self.prev[0]:
            rate = (err - self.prev[1]) / (now - self.prev[0])
        self.prev = (now, err)
        tau = max(-self.tmax, min(self.tmax, self.kp * err - self.kd * rate)) * self.sign
        for pub in self.roll_pubs:
            pub.publish(Float64(data=float(tau)))
        self.err_pub.publish(Float64(data=math.degrees(err)))
        if state != self.state:
            self.state = state
            self.get_logger().info(f'-> {state} (머리 s={s_head:.2f}, 꼬리 s={s_tail:.2f}, 굽힘 평면 오차 {math.degrees(err):+.0f}°)')
        self.state_pub.publish(String(data=state))
        self.get_logger().info(f'[{state}] 굽힘 평면 오차 {math.degrees(err):+.1f}°, 굴림 토크 {tau:+.2f}',
                               throttle_duration_sec=3.0)


def main():
    rclpy.init()
    rclpy.spin(AiroCtrl())


if __name__ == '__main__':
    main()
