"""모델 x 배관 일괄 주행 비교 (화면 없이) — 실물 제작 기준 평가 포함.

ros2 run pipe_robot_sim bench --models three_segment pirate_v --courses straight elbow90 \
    --fittings "" dv_ll90 -o result.csv

각 조합마다 sim.launch.py 를 헤드리스로 띄우고 /cmd_vel 로 일정 속도 전진.
끝(마개)에 도착하거나, 15 초(sim) 동안 진행이 없거나, 시간 초과면 종료.

기록 (실물 기준)
  align_max  : 마디(bench_spines)가 자기 위치의 배관 방향과 이루는 각도 최댓값 [deg] — 코너에서도 일자인지
  roll_max   : 몸통이 배관 축 둘레로 비틀린 각도 최댓값 [deg] (배관을 따라 옮긴 위쪽 기준) — 뒤틀림
  sat_s      : 스프링 슬라이더·관절이 범위 끝(2% 이내)에 닿아 있던 시간 [s] — 바퀴가 휘거나 끼임
  slip_pct   : (엔코더 거리 - 실제 거리) / 실제 거리 [%] — 헛돎·미끄러짐
  stall_s    : 진행 속도가 목표의 30% 아래였던 시간 [s] — 걸림
  real_ok    : PASS + align_max<=15 + roll_max<=10 + sat_s<=0.5 + |slip|<=10 + stall_s<=1
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray, String
from tf2_msgs.msg import TFMessage

from pipe_robot_sim.generate_world import default_courses_file, overrides
from pipe_robot_sim.joint_follow import quat_to_rot
from pipe_robot_sim.pipe_geometry import build_course, load_config

FIT90 = {'dv90', 'dv_ll90', 'ts90'}
LIMITS = dict(align=15.0, roll=10.0, sat=0.5, slip=10.0, stall=1.0)


def rpy(o):
    w, x, y, z = o.w, o.x, o.y, o.z
    return (math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x))))))


class Runner(Node):
    def __init__(self):
        super().__init__('bench', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reset()
        self.create_subscription(Odometry, 'ground_truth/odom', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(Odometry, 'odom', lambda m: setattr(self, 'est', m), 10)
        self.create_subscription(TFMessage, 'gz_poses', self.on_poses, 10)
        self.create_subscription(JointState, 'joint_states',
                                 lambda m: self.js.update(dict(zip(m.name, m.position))), 10)
        self.create_subscription(Int64MultiArray, 'wheel_encoders', lambda m: setattr(self, 'enc', list(m.data)), 10)
        self.create_subscription(String, 'robot_description', self.on_desc,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def reset(self):
        self.gt = self.est = self.enc = None
        self.poses, self.js, self.limits = {}, {}, {}

    def on_poses(self, msg):
        for t in msg.transforms:
            tr, r = t.transform.translation, t.transform.rotation
            self.poses[t.child_frame_id] = (np.array([tr.x, tr.y, tr.z]), quat_to_rot((r.w, r.x, r.y, r.z)))

    def on_desc(self, msg):
        for j in ET.fromstring(msg.data).iter('joint'):
            lim = j.find('limit')
            if j.get('type') in ('revolute', 'prismatic') and lim is not None:
                self.limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))

    def link_world(self, name):
        pm, Rm = self.poses['pipe_robot']
        pl, Rl = self.poses[name]
        return pm + Rm @ pl, Rm @ Rl

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def model_params(model):
    p = os.path.join(get_package_share_directory('pipe_robot_sim'), 'models', model, 'robot.yaml')
    with open(p, encoding='utf-8') as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def kill_marked():
    """PIPE_BENCH 표식이 붙은 프로세스(이전 벤치의 남은 시뮬 포함) 모두 종료.
    남아 있던 시뮬이 같은 ROS 토픽에 옛 자세를 계속 내보내 결과를 망가뜨렸음."""
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = f.read()
        except OSError:
            continue
        if b'PIPE_BENCH=' in env:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


_RUN = [0]


def run_one(node, model, course, fitting, speed, cfg, log_dir):
    fit = overrides(fitting if fitting in FIT90 else '', fitting if fitting and fitting not in FIT90 else '')
    b = build_course(cfg, course, fit)
    end = b.pos
    path = np.array([q[0] for q in b.path])
    tang = np.array([q[1] for q in b.path])
    ups = np.array([q[2] for q in b.path])
    seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    mp_ = model_params(model)
    spines = mp_.get('bench_spines', ['base_link'])
    wr, cpr = mp_.get('wheel_radius', 0.012), mp_.get('encoder_cpr', 2800)

    def nearest(p):
        i = int(np.argmin(np.linalg.norm(path - p, axis=1)))
        return seg[i], tang[i]

    def twist(p, R):
        """몸통 위쪽(z)이 배관을 따라 옮겨온 위쪽 기준에서 축 둘레로 돌아간 각도 [deg] (수직관에서도 의미 있음)"""
        i = int(np.argmin(np.linalg.norm(path - p, axis=1)))
        t, u = tang[i], ups[i]
        z = R[:, 2] - np.dot(R[:, 2], t) * t
        if np.linalg.norm(z) < 1e-6:
            return 0.0
        z /= np.linalg.norm(z)
        return math.degrees(math.atan2(float(np.dot(np.cross(u, z), t)), float(np.dot(u, z))))

    args = ['ros2', 'launch', 'pipe_robot_sim', 'sim.launch.py', f'course:={course}', f'model:={model}',
            'gui:=false', 'rviz:=false']
    if fit:
        args += [f'fitting90:={fitting}'] if fitting in FIT90 else [f'fitting45:={fitting}']
    log = open(os.path.join(log_dir, f'{model}_{course}_{fitting or "base"}.log'), 'w')
    kill_marked()                       # 이전 실행의 찌꺼기 제거 후 시작
    _RUN[0] += 1
    env = dict(os.environ, PIPE_BENCH=str(_RUN[0]))
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
    node.reset()
    res = dict(model=model, course=course, fitting=fitting or '-', result='ERROR', real_ok='', time_s='',
               progress='', align_max='', align_where='', roll_max='', sat_s='', sat_joint='', slip_pct='',
               stall_s='', est_err_mm='', note='')
    try:
        t_wall = time.time()
        while (node.gt is None or 'pipe_robot' not in node.poses) and time.time() - t_wall < 40:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.gt is None:
            res['note'] = '시작 실패'
            return res
        time.sleep(1.0)
        t0 = node.now()
        t_prev = t0
        limit = seg[-1] / speed * 2.5 + 20
        best_s, last_prog_t = 0.0, t0
        roll_max = align_max = sat = stall = 0.0
        align_where, sat_cnt = '', {}
        hist = []                       # (t, s) 진행 속도 계산용
        enc0 = None
        true_dist, prev_pos = 0.0, None
        result = 'TIMEOUT'
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.pub.publish(Twist(linear=Vector3(x=speed)))
            tn = node.now()
            dt, t_prev = tn - t_prev, tn
            t = tn - t0
            p = node.gt.pose.pose.position
            pos = np.array([p.x, p.y, p.z])
            s, _ = nearest(pos)
            o = node.gt.pose.pose.orientation
            r = twist(pos, quat_to_rot((o.w, o.x, o.y, o.z)))
            roll_max = max(roll_max, abs(r))
            if prev_pos is not None:
                true_dist += float(np.linalg.norm(pos - prev_pos))
            prev_pos = pos
            if node.enc is not None and enc0 is None:
                enc0 = list(node.enc)
            # 마디 정렬
            for sp in spines:
                try:
                    if ':' in sp:
                        a, bb = sp.split(':')
                        pa, pb = node.link_world(a)[0], node.link_world(bb)[0]
                        v, mid = pa - pb, (pa + pb) / 2
                    else:
                        mid, R = node.link_world(sp)
                        v = R[:, 0]
                except KeyError:
                    continue
                _, tg = nearest(mid)
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(v / np.linalg.norm(v), tg))))))
                if ang > align_max:
                    align_max, align_where = ang, f'{sp}@{nearest(mid)[0]:.2f}m'
            # 범위 끝 접촉
            if dt > 0:
                for j, (lo, hi) in node.limits.items():
                    q = node.js.get(j)
                    if q is None:
                        continue
                    m = 0.02 * (hi - lo)
                    if q < lo + m or q > hi - m:
                        sat_cnt[j] = sat_cnt.get(j, 0.0) + dt
                sat = max(sat_cnt.values(), default=0.0)
            # 정체
            hist.append((tn, s))
            while hist and tn - hist[0][0] > 0.5:
                hist.pop(0)
            if t > 3.0 and len(hist) > 2 and hist[-1][0] - hist[0][0] > 0.3:
                v_prog = (hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0])
                if v_prog < 0.3 * speed:
                    stall += dt
            if s > best_s + 0.005:
                best_s, last_prog_t = s, tn
            if np.linalg.norm(pos - end) < 0.2:
                result = 'PASS'
                break
            if tn - last_prog_t > 15:
                result = 'STUCK'
                break
            if t > limit:
                break
            if pos[2] < -0.05 or np.linalg.norm(pos[:2]) > 10:
                result = 'FLIPPED'
                break
        slip = ''
        if enc0 is not None and node.enc is not None and true_dist > 0.05:
            enc_d = float(np.mean(np.array(node.enc) - np.array(enc0))) * 2 * math.pi / cpr * wr
            slip = (enc_d - true_dist) / true_dist * 100
        sat_joint = max(sat_cnt, key=sat_cnt.get) if sat_cnt else ''
        ok = (result == 'PASS' and align_max <= LIMITS['align'] and roll_max <= LIMITS['roll']
              and sat <= LIMITS['sat'] and (slip == '' or abs(slip) <= LIMITS['slip']) and stall <= LIMITS['stall'])
        res.update(result=result, real_ok='O' if ok else 'X', time_s=f'{t:.1f}',
                   progress=f'{best_s / seg[-1] * 100:.0f}%', align_max=f'{align_max:.1f}', align_where=align_where,
                   roll_max=f'{roll_max:.1f}', sat_s=f'{sat:.1f}', sat_joint=sat_joint,
                   slip_pct='' if slip == '' else f'{slip:.1f}', stall_s=f'{stall:.1f}')
        if node.est is not None:
            e = node.est.pose.pose.position
            res['est_err_mm'] = f'{np.linalg.norm(np.array([e.x, e.y, e.z]) - pos) * 1000:.0f}'
        return res
    finally:
        node.pub.publish(Twist())
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=8)
        except Exception:
            pass
        time.sleep(1)
        kill_marked()                   # gazebo 등 자식까지 전부 종료
        log.close()
        time.sleep(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', nargs='+', default=['three_wheel', 'three_segment', 'pirate_v', 'tracked'])
    ap.add_argument('--courses', nargs='+', default=['straight', 'elbow45', 'elbow90', 'demo'])
    ap.add_argument('--fittings', nargs='+', default=[''], help='"" = 코스 기본, 예: dv_ll90 ts90')
    ap.add_argument('--speed', type=float, default=0.06)
    ap.add_argument('-o', '--out', default='bench_result.csv')
    a = ap.parse_args()

    # ToF(gpu_lidar)는 렌더링이 필요 -> 화면이 있는 PC면 그 디스플레이를 빌림 (창은 안 뜸)
    if 'DISPLAY' not in os.environ and os.path.exists('/tmp/.X11-unix/X0'):
        os.environ['DISPLAY'] = ':0'
        xa = f'/run/user/{os.getuid()}/gdm/Xauthority'
        if os.path.exists(xa):
            os.environ.setdefault('XAUTHORITY', xa)
    cfg = load_config(default_courses_file())
    log_dir = os.path.splitext(a.out)[0] + '_logs'
    os.makedirs(log_dir, exist_ok=True)
    rclpy.init()
    node = Runner()
    rows = []
    combos = [(m, c, f) for m in a.models for c in a.courses for f in a.fittings
              if not f or (c in ('elbow90', 'demo') and f in FIT90) or (c in ('elbow45', 'demo') and f not in FIT90)]
    for i, (m, c, f) in enumerate(combos, 1):
        r = run_one(node, m, c, f, a.speed, cfg, log_dir)
        if r['result'] == 'ERROR' or (r['result'] == 'PASS' and r['slip_pct'] == ''):
            # 측정 이상 (시작 실패, 엔코더·자세 기록 없음) -> 한 번 다시
            r = run_one(node, m, c, f, a.speed, cfg, log_dir)
            r['note'] = (r['note'] + ' 재측정').strip()
        rows.append(r)
        print(f'[{i}/{len(combos)}] {m:14s} {c:9s} {r["fitting"]:8s} {r["result"]:7s} 실물OK {r["real_ok"]:1s} '
              f'정렬max {r["align_max"]:>5s}° 롤max {r["roll_max"]:>5s}° 한계접촉 {r["sat_s"]:>4s}s({r["sat_joint"]}) '
              f'미끄럼 {r["slip_pct"]:>5s}% 정체 {r["stall_s"]:>4s}s 추정오차 {r["est_err_mm"]:>4s}mm '
              f'| 최대정렬 위치 {r["align_where"]} {r["note"]}', flush=True)
        with open(a.out, 'w', newline='', encoding='utf-8') as fo:
            w = csv.DictWriter(fo, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
