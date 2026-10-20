"""뱀형 V자 (snake_v) 관절 제어.

gait_mode
  clamp  : 앞뒤 V 두 개로 위아래 벽을 눌러 버팀 (직관 주행 자세). 가운데 링크와 머리는 배관 축과 나란히.
  oracle : clamp 로 가다가 엘보마다 PIRATE 방식 순서로 통과. 배관 모양과 정답 자세를 쓰므로 실물에서는
           불가능 -> "이 구조로 엘보를 지날 수 있는가"의 상한 확인용. 단계:
     CRUISE    두 V 누름
     축 회전    머리가 엘보 앞 align_before -> 몸 전체를 배관 축 둘레로 돌려 V 가 굽는 평면을 엘보 평면에 맞춤
               (PIRATE 논문의 axial rotation, Dertien 2014 4.4절). 앞뒤를 나눠서:
       A_FREE_F  바퀴 멈춤, 앞 V 풂 (뒤 V가 버팀)
       A_ROT_F   J3 를 돌려 앞부분을 필요한 각도만큼 회전
       A_CLAMP_F 앞 V 누름
       A_FREE_R  뒤 V 풂 (앞 V가 버팀)
       A_ROT_R   J3 를 0 으로 -> 뒷부분이 같은 각도만큼 회전
       A_CLAMP_R 뒤 V 누름 -> CRUISE
               (앞부분만 돌리면 뒷부분 J2 가 엘보 방향으로 못 꺾여 가운데 링크가 모서리에 걸림)
     F_RELEASE 머리가 엘보 앞 release_before -> 바퀴 멈춤, 앞 V를 풀어 F_PASS 와 같은 겨냥 자세로
               (일자로 펴면 머리가 엘보 바깥 벽에 박혀 관절이 못 움직임)
     F_PASS    뒤 V는 누름 유지, 가운데 링크와 앞부분은 '바깥 벽 선'을 따르는 모양 (follow-the-leader, fit_front):
               J2 위치에서 선을 따라 링크 길이 간격으로 J4, J5, J6 자리를 잡고 이웃 링크 사이 각을 관절 목표로.
               바깥 벽 선 = 중심선에서 엘보 바깥쪽으로 h (바퀴 중심이 벽에 닿는 높이). 앞바퀴가 바깥 벽을 타고 같이 밂.
               (중심선을 따르면 앞부분이 배관 가운데 떠서 앞바퀴가 힘을 못 냄)
               가운데 링크도 J2 로 돌림 (PIRATE 의 'bend payload').
               안 돌리면 가운데 링크 앞끝 바퀴가 출구 쪽 벽에 정면으로 박혀 멈춤 (2026-10-20).
               (press_mm: 겨냥선을 벽 너머로 잡는 여유. 6 mm 로 해 보니 직관에서부터 마찰이 커져 더 나빠짐 -> 0)
     F_CLAMP   앞 V 다시 누름. 시점 = 먼저 오는 것: (1) J4 가 엘보 끝을 4 cm 지남 (앞 V 전체가 다음 직관;
               바로 누르면 V 를 접는 힘에 앞 V 가 모서리로 끌려옴) (2) 뒤 V 앞 관절(J2)이 엘보에 5 cm 들어감
               (더 밀면 뒤 V가 엘보에 끼임 -> 곡선이 긴 대곡 엘보)
               (3) 2 초 동안 진행이 없고 J5 가 엘보를 지남 (가운데 링크가 모서리에 걸림: 뒷부분이 아직 안 돌아
                   J2 가 엘보 방향으로 못 꺾임 -> 앞 V로 넘겨받고 뒷부분을 돌려야 풀림)
               J4 는 엘보 안에 있을 수 있어 약하게 둠. 너무 일찍 누르면 앞 V가 엘보 모서리에 L 자로 걸림
     R_RELEASE 바퀴 멈춤, 뒤 V 풂, J3 를 0 으로 (뒷부분을 앞부분 평면에 맞춤, 앞 V가 버팀)
     R_PASS    앞 V가 끌고 감. J4 위치에서 바깥 벽 선을 따라 뒤로 J2, J1 자리를 잡음 (fit_rear)
관절 목표는 초당 rate_limit_deg 이하로만 바뀌게 함 (상태가 바뀔 때 목표가 한 번에 튀지 않게).
     R_CLAMP   맨 뒤 바퀴가 엘보를 지나면 뒤 V 다시 누름 -> CRUISE

V 팔 각도 θ: 아래 바퀴와 위 바퀴 중심의 높이 차 = 팔 길이 x sin θ.
  바퀴(구, 반경 rw)가 좌우 yw 에 있으므로 바퀴 중심이 벽에 닿는 높이는 ±sqrt((Rp - rw)^2 - yw^2).
관절 부호 (snake_v.urdf.xacro): 앞쪽 사슬은 +q 면 앞 끝이 내려감, 뒤쪽 사슬은 +q 면 뒤 끝이 올라감.
  앞 V: J4 = -θ (팔이 위로), J5 = +2θ + δ (꼭짓점, 더 굽혀 위 벽을 누름), J6 = -θ (머리 수평)
  뒤 V: J2 = +θ, J1 = -2θ - δ
δ (clamp_extra_deg): 꼭짓점을 실제보다 더 굽히라고 명령해 서보가 토크 한계(tmax)로 계속 누르게 함.
약한 관절: 서보 목표 = 현재각 + soft_ratio x (원하는 각 - 현재각) -> 서보 P 게인 x soft_ratio 짜리 약한 스프링.

sub /joint_states, /gz_poses (oracle)
pub /servo/<관절> (rad), /wheel_dirs (바퀴별 속도 배수, motor_driver), /snake_gait/state (상태 이름)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String
from tf2_msgs.msg import TFMessage

from pipe_robot_sim.generate_world import default_courses_file, overrides
from pipe_robot_sim.odometry import quat_to_rot
from pipe_robot_sim.pipe_geometry import build_course, load_config

JOINTS = ['j1_joint', 'j2_joint', 'j3_joint', 'j4_joint', 'j5_joint', 'j6_joint']
WHEELS = ['w1', 'w2', 'w3', 'w4', 'w5', 'w6']        # 각 2개 (_l, _r), robot.yaml wheel_joints 순서와 같음
ROBOT = 'pipe_robot'


def unit(v):
    return v / np.linalg.norm(v)


class SnakeGait(Node):
    def __init__(self):
        super().__init__('snake_gait')
        d = self.declare_parameter
        self.mode = d('gait_mode', 'clamp').value
        self.L = d('arm_len', 0.085).value
        self.Lh = d('head_len', 0.05).value
        rp = d('pipe_inner_d', 0.1008).value / 2
        rw = d('wheel_radius', 0.015).value
        yw = d('wheel_y', 0.016).value
        self.extra = math.radians(d('clamp_extra_deg', 10.0).value)
        self.soft = d('soft_ratio', 0.1).value
        self.pass_speed = d('pass_speed_ratio', 0.6).value
        self.pre = d('release_before', 0.08).value        # 머리가 엘보 곡선 시작 전 이 거리에서 앞 V 풂 [m]
        self.align_pre = d('align_before', 0.25).value    # 이 거리에서 몸 전체 축 회전 [m]
        self.aligned_i = -1                               # 축 회전을 마친 엘보 번호
        self.after = d('clamp_after', 0.02).value         # 엘보 끝을 이만큼 지나면 다시 누름 [m]
        h = math.sqrt((rp - rw) ** 2 - yw ** 2)          # 바퀴 중심이 벽에 닿는 높이
        self.h = h + d('press_mm', 0.0).value / 1000.0   # 겨냥선: 바퀴가 벽에 닿는 높이 + 누르는 여유
        self.Lm = d('mid_len', 0.10).value
        self.outer, self.outer_k = None, None            # 바깥 벽 선 (엘보마다)
        self.rate = math.radians(d('rate_limit_deg', 60.0).value)
        self.cmd = {}                                     # 마지막으로 보낸 관절 목표 (변화 속도 제한용)
        self.theta = math.asin(min(1.0, 2 * h / self.L))

        self.q = {}
        self.poses = {}
        self.state = 'CRUISE'
        self.t_state = None
        self.q3_goal = 0.0
        self.elbow_i = 0
        self.prog = None
        self.pubs = {j: self.create_publisher(Float64, f'servo/{j}', 10) for j in JOINTS}
        self.dirs_pub = self.create_publisher(Float64MultiArray, 'wheel_dirs', 10)
        self.state_pub = self.create_publisher(String, 'snake_gait/state', 10)
        self.qd = {}
        self.press_t = None                               # 꼭짓점이 벽에 막혀 누르기 시작한 시각
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        if self.mode == 'oracle':
            self.create_subscription(TFMessage, 'gz_poses', self.on_poses, 10)
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
                self.elbows.append(dict(name=name, s0=s0, s_bend=s0 + leg, s1=s0 + 2 * leg + f['R'] * ang, axis=axis))
            self.get_logger().info('엘보: ' + ', '.join(f"{e['name']} {e['s0']:.2f}~{e['s1']:.2f} m" for e in self.elbows))
        self.create_timer(0.02, self.step)
        self.get_logger().info(f'mode={self.mode}, V 팔 각도 θ = {math.degrees(self.theta):.1f}°, 누름 여유 δ = '
                               f'{math.degrees(self.extra):.0f}°')

    def on_js(self, m):
        self.q.update(dict(zip(m.name, m.position)))
        self.qd.update(dict(zip(m.name, m.velocity)))

    # ---------- 배관 / 자세 ----------
    def idx(self, s):
        return int(np.clip(np.searchsorted(self.seg, s), 0, len(self.seg) - 1))

    def s_of(self, p):
        return float(self.seg[int(np.argmin(np.linalg.norm(self.pts - p, axis=1)))])

    def on_poses(self, msg):
        for t in msg.transforms:
            tr, r = t.transform.translation, t.transform.rotation
            self.poses[t.child_frame_id] = (np.array([tr.x, tr.y, tr.z]), quat_to_rot((r.w, r.x, r.y, r.z)))

    def link(self, name, offset=(0.0, 0.0, 0.0)):
        pm, Rm = self.poses[ROBOT]
        pl, Rl = self.poses[name]
        R = Rm @ Rl
        return pm + Rm @ pl + R @ np.array(offset), R

    # ---------- 관절 목표 ----------
    def clamp_front(self):
        t, e = self.theta, self.extra
        return {'j4_joint': -t, 'j5_joint': 2 * t + e, 'j6_joint': -t}

    def clamp_rear(self):
        t, e = self.theta, self.extra
        return {'j1_joint': -2 * t - e, 'j2_joint': t}

    def outer_line(self):
        """현재 엘보의 바깥 벽 선 = 중심선 + h x (엘보 바깥 방향). 바퀴 중심이 바깥 벽에 닿는 선."""
        k = min(self.elbow_i, len(self.elbows) - 1)
        if self.outer_k != k:
            self.outer = self.pts - self.h * np.cross(self.elbows[k]['axis'], self.tang)
            self.outer_k = k
        return self.outer

    def along(self, p, dist):
        """바깥 벽 선 위에서 p 와 직선거리 |dist| 인 점 (dist > 0 앞, < 0 뒤). 점 사이(5~7 mm)는 보간."""
        line = self.outer_line()
        i = int(np.argmin(np.linalg.norm(line - p, axis=1)))
        seg = line[i:] if dist > 0 else line[i::-1]
        dd = np.linalg.norm(seg - p, axis=1)
        k = np.nonzero(dd >= abs(dist))[0]
        if len(k) == 0 or k[0] == 0:
            return seg[-1] if len(k) == 0 else seg[0]
        k = k[0]
        f = (abs(dist) - dd[k - 1]) / max(dd[k] - dd[k - 1], 1e-9)
        return seg[k - 1] + f * (seg[k] - seg[k - 1])

    def bend_y(self):
        """굽힘 축 (링크 y) = 엘보 회전축 ±a. 부호만 실제 base_link 자세에서 (축 회전으로 맞춰 둠)."""
        a = self.elbows[min(self.elbow_i, len(self.elbows) - 1)]['axis']
        return a if self.link('base_link')[1][:, 1] @ a > 0 else -a

    @staticmethod
    def ang(u, v, y):
        """링크 방향 u -> v 회전각 (축 y). 관절각 = 부모 링크 방향 -> 자식 링크 방향 (방향은 모두 앞쪽을 향하게)."""
        return math.atan2(float(np.dot(np.cross(u, v), y)), float(np.dot(u, v)))

    def fit_front(self):
        """뒤 V가 버티는 동안 J2, J4~J6 목표 (follow-the-leader).
        J2 위치에서 바깥 벽 선을 따라 링크 길이 간격으로 J4, J5, J6, 머리 끝 자리를 잡고, 이웃 링크 방향 사이 각을 관절각으로.
        실제 자세는 J2 위치와 r2 방향만 씀 (누른 뒤 V 라 안정). 링크마다 현재 자세로 겨냥하면 관절끼리 서로의
        움직임을 다시 받아 꿈틀거림 (±25°, 0.5 Hz, 2026-10-20)."""
        p2, Rr2 = self.link('r2')
        y = self.bend_y()
        p4 = self.along(p2, self.Lm)
        p5 = self.along(p4, self.L)
        p6 = self.along(p5, self.L)
        p7 = self.along(p6, self.Lh)
        dm, d1, d2, dh = (unit(b - a) for a, b in ((p2, p4), (p4, p5), (p5, p6), (p6, p7)))
        return {'j2_joint': self.ang(Rr2[:, 0], dm, y), 'j4_joint': self.ang(dm, d1, y),
                'j5_joint': self.ang(d1, d2, y), 'j6_joint': self.ang(d2, dh, y)}

    def fit_rear(self):
        """앞 V가 버티는 동안 J4, J2, J1 목표. J4 위치에서 바깥 벽 선을 따라 뒤로 J2, J1, 꼬리 자리를 잡음."""
        p4, Rf1 = self.link('f1')
        y = self.bend_y()
        p2 = self.along(p4, -self.Lm)
        p1 = self.along(p2, -self.L)
        p0 = self.along(p1, -self.L)
        dm, d2, d1 = (unit(b - a) for a, b in ((p2, p4), (p1, p2), (p0, p1)))
        return {'j4_joint': self.ang(dm, Rf1[:, 0], y), 'j2_joint': self.ang(dm, d2, y), 'j1_joint': self.ang(d2, d1, y)}

    def near(self, targets, deg):
        return all(abs(self.q[j] - v) < math.radians(deg) for j, v in targets.items())

    def clamped(self, j, sign, dt):
        """꼭짓점 관절(앞 J5 부호 +, 뒤 J1 부호 -)이 V 로 벽을 잡았나.
        (1) 거의 다 접힘 (2θ - 8° 이상) 또는
        (2) 절반(θ) 넘게 접혔고, 명령보다 5° 넘게 덜 접힌 채 0.5 초 멈춰 있음 = 벽에 막혀 누르는 중.
        V 를 눕히면 중력 때문에 다 잡아도 덜 접힐 수 있어 (1) 만으로는 못 넘어감 (2026-10-20). 오래 걸리면 경고만."""
        q, cmd = sign * self.q[j], sign * self.cmd.get(j, self.q[j])
        pressing = q > self.theta and cmd - q > math.radians(5) and abs(self.qd.get(j, 0.0)) < 0.05
        if pressing:
            self.press_t = self.press_t or self.now()
        else:
            self.press_t = None
        ok = q > 2 * self.theta - math.radians(8) or (self.press_t is not None and self.now() - self.press_t > 0.5)
        if not ok and dt > 8:
            self.get_logger().warn(f'[{self.state}] {j} 가 아직 덜 접힘 ({math.degrees(self.q[j]):.0f}°), 기다리는 중',
                                   throttle_duration_sec=5.0)
        return ok

    def soft_to(self, j, goal):
        q = self.q.get(j, 0.0)
        return q + self.soft * (goal - q)

    def publish(self, targets, dirs, speed=1.0):
        dt = 0.02
        for j, v in targets.items():
            prev = self.cmd.get(j, self.q.get(j, v))
            v = prev + max(-self.rate * dt, min(self.rate * dt, v - prev))
            self.cmd[j] = v
            self.pubs[j].publish(Float64(data=float(v)))
        if self.state not in ('CRUISE',):
            self.get_logger().info(
                f'[{self.state}] 명령 ' + ' '.join(f'{j[:2]}={math.degrees(v):.0f}' for j, v in sorted(targets.items()))
                + ' | 실제 ' + ' '.join(f'{j[:2]}={math.degrees(self.q.get(j, 0)):.0f}' for j in sorted(targets)),
                throttle_duration_sec=2.0)
        self.dirs_pub.publish(Float64MultiArray(data=[float(dirs[w] * speed) for w in WHEELS for _ in (0, 1)]))
        self.state_pub.publish(String(data=self.state))

    def go(self, state):
        self.prog = None
        self.press_t = None
        self.state = state
        self.t_state = self.now()
        self.get_logger().info(f'-> {state}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 주기 ----------
    def step(self):
        if not all(j in self.q for j in JOINTS):
            return
        clamp_dirs = {w: 1.0 for w in WHEELS}     # 꼭짓점 바퀴(w2, w5)는 축이 뒤집혀 있어 +1 로 위 벽에서 앞으로 감
        stop = {w: 0.0 for w in WHEELS}           # 제동 (motor_driver 가 속도 0 을 붙잡음)
        # V 를 누르는 동안: 누르는 쪽 바퀴는 자유 회전 (벽을 따라 미끄러져야 함), 버티는 쪽은 제동
        # (모두 제동하면 V 를 접는 힘에 몸 전체가 끌려감, 2026-10-20)
        nan = float('nan')
        grip_front = {**stop, 'w4': nan, 'w5': nan, 'w6': nan}
        grip_rear = {**stop, 'w1': nan, 'w2': nan, 'w3': nan}
        if self.mode != 'oracle':
            self.publish({**self.clamp_rear(), 'j3_joint': 0.0, **self.clamp_front()}, clamp_dirs)
            return
        if ROBOT not in self.poses:
            return
        if self.t_state is None:
            self.t_state = self.now()
        el = self.elbows[self.elbow_i] if self.elbow_i < len(self.elbows) else None
        try:
            head = self.s_of(self.link('head', (self.Lh, 0, 0))[0])
            s_j4 = self.s_of(self.link('f1')[0])
            s_j5 = self.s_of(self.link('f2')[0])
            s_j2 = self.s_of(self.link('r2')[0])
            s_w1 = self.s_of(self.link('r1', (-self.L, 0, 0))[0])
            _, Rb = self.link('base_link')
        except KeyError:
            return
        dt = self.now() - self.t_state
        q3 = self.q['j3_joint']
        soft_front = {j: self.soft_to(j, 0.0) for j in ('j4_joint', 'j5_joint', 'j6_joint')}
        soft_rear = {j: self.soft_to(j, 0.0) for j in ('j1_joint', 'j2_joint')}

        if self.state == 'CRUISE':
            self.publish({**self.clamp_rear(), 'j3_joint': self.q3_goal, **self.clamp_front()}, clamp_dirs)
            if el is not None and self.aligned_i != self.elbow_i and head > el['s_bend'] - self.align_pre:
                # 앞부분 굽힘 축(base_link y)을 -a (엘보 회전축 반대)에 맞추는 회전각 φ (배관 축 둘레).
                # 그러면 base_link 아래(-z, 가운데 링크가 닿는 벽)가 엘보 바깥 벽 -> 바깥 벽 선을 따르는 F_PASS 와 맞음.
                # (±a 중 작은 쪽을 고르면 왼쪽 엘보는 ±90° 동률이라 수치 오차로 정해졌음, 2026-10-20)
                # 왼쪽 -90°, 오른쪽 +90°, 위 0°, 아래 180°
                x, y, a = Rb[:, 0], Rb[:, 1], el['axis']
                self.phi = math.atan2(float(np.dot(np.cross(y, -a), x)), float(np.dot(y, -a)))
                self.aligned_i = self.elbow_i
                if abs(self.phi) > math.radians(10):
                    # 뒤 V가 고정이면 J3 +Δ -> 앞부분 -Δ 회전
                    self.q3_goal = q3 - self.phi
                    self.get_logger().info(f"{el['name']}: 몸 전체 {math.degrees(self.phi):.0f}° 회전 "
                                           f"(J3 {math.degrees(q3):.0f}° -> {math.degrees(self.q3_goal):.0f}° -> 0°)")
                    self.go('A_FREE_F')
            elif el is not None and self.aligned_i == self.elbow_i and head > el['s_bend'] - self.pre:
                self.go('F_RELEASE')
        elif self.state == 'A_FREE_F':
            free = {j: self.soft_to(j, 0.0) for j in ('j4_joint', 'j5_joint', 'j6_joint')}
            self.publish({**self.clamp_rear(), 'j3_joint': q3, **free}, stop)
            if abs(self.q['j5_joint']) < math.radians(40) or dt > 3:
                self.go('A_ROT_F')
        elif self.state == 'A_ROT_F':
            free = {j: self.soft_to(j, 0.0) for j in ('j4_joint', 'j5_joint', 'j6_joint')}
            self.publish({**self.clamp_rear(), 'j3_joint': self.q3_goal, **free}, stop)
            if abs(q3 - self.q3_goal) < math.radians(3) or dt > 6:
                self.go('A_CLAMP_F')
        elif self.state == 'A_CLAMP_F':
            self.publish({**self.clamp_rear(), 'j3_joint': self.q3_goal, **self.clamp_front()}, grip_front)
            if self.clamped('j5_joint', +1, dt):     # 앞 V가 잡기 전에는 뒤 V를 풀지 않음
                self.go('A_FREE_R')
        elif self.state == 'A_FREE_R':
            free = {j: self.soft_to(j, 0.0) for j in ('j1_joint', 'j2_joint')}
            self.publish({**free, 'j3_joint': self.q3_goal, **self.clamp_front()}, stop)
            if abs(self.q['j1_joint']) < math.radians(40) or dt > 3:
                self.q3_goal = 0.0
                self.go('A_ROT_R')
        elif self.state == 'A_ROT_R':
            free = {j: self.soft_to(j, 0.0) for j in ('j1_joint', 'j2_joint')}
            self.publish({**free, 'j3_joint': self.q3_goal, **self.clamp_front()}, stop)
            if abs(q3 - self.q3_goal) < math.radians(3) or dt > 6:
                self.go('A_CLAMP_R')
        elif self.state == 'A_CLAMP_R':
            self.publish({**self.clamp_rear(), 'j3_joint': self.q3_goal, **self.clamp_front()}, grip_rear)
            if self.clamped('j1_joint', -1, dt) or dt > 8:
                self.go('CRUISE')
        elif self.state == 'F_RELEASE':
            fit = self.fit_front()
            front = {j: fit[j] for j in ('j4_joint', 'j5_joint', 'j6_joint')}
            self.publish({**self.clamp_rear(), 'j3_joint': self.q3_goal, **front}, stop)
            if self.near(front, 10) or dt > 5:
                self.go('F_PASS')
        elif self.state == 'F_PASS':
            dirs = {**clamp_dirs, 'w5': -1.0}     # 앞부분이 펴져 w5 도 아래(바깥) 벽에 닿음 -> 방향 반대
            self.publish({'j1_joint': self.clamp_rear()['j1_joint'], 'j3_joint': self.q3_goal, **self.fit_front()},
                         dirs, self.pass_speed)
            if self.prog is None or head > self.prog[0] + 0.005:
                self.prog = (head, self.now())            # 진행 기록 (머리 위치, 시각)
            stalled = self.now() - self.prog[1] > 2.0
            if s_j4 > el['s1'] + 0.04 or s_j2 > el['s_bend'] + 0.05 or (stalled and s_j5 > el['s1']):
                self.get_logger().info(f'앞 V 누름: J4 s={s_j4:.3f} J5 s={s_j5:.3f} (엘보 끝 {el["s1"]:.3f}), '
                                       f'J2 s={s_j2:.3f}, 정체 {stalled}')
                self.go('F_CLAMP')
        elif self.state == 'F_CLAMP':
            front = self.clamp_front()
            front['j4_joint'] = self.soft_to('j4_joint', -self.theta)
            self.publish({**self.clamp_rear(), 'j3_joint': self.q3_goal, **front}, grip_front)
            if self.clamped('j5_joint', +1, dt):     # 앞 V가 잡기 전에는 뒤 V를 풀지 않음
                self.go('R_RELEASE')
        elif self.state == 'R_RELEASE':
            fit = self.fit_rear()
            rear = {j: fit[j] for j in ('j1_joint', 'j2_joint')}
            self.publish({**rear, 'j3_joint': self.q3_goal, **self.clamp_front()}, stop)
            if self.near(rear, 10) or dt > 5:
                self.go('R_PASS')
        elif self.state == 'R_PASS':
            dirs = {**clamp_dirs, 'w2': -1.0}     # 뒷부분이 펴져 w2 도 아래 벽에 닿음
            self.publish({**self.clamp_front(), 'j3_joint': self.q3_goal, **self.fit_rear()}, dirs, self.pass_speed)
            if s_w1 > el['s1'] + self.after:
                self.go('R_CLAMP')
        elif self.state == 'R_CLAMP':
            self.publish({**self.clamp_rear(), 'j3_joint': self.q3_goal, **self.clamp_front()}, grip_rear)
            if self.clamped('j1_joint', -1, dt) or dt > 8:
                self.elbow_i += 1
                self.go('CRUISE')


def main():
    rclpy.init()
    rclpy.spin(SnakeGait())


if __name__ == '__main__':
    main()
