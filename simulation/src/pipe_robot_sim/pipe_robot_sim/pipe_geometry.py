"""배관 코스 -> 박스 스테이브(벽 조각) 목록.

배관 벽을 원주 방향으로 N개의 얇은 박스로 근사한다 (메시보다 접촉이 안정적).
코스는 중심선을 따라가는 '거북이'로 직관/엘보를 이어 붙인다.
세그먼트마다 내경을 따로 두므로 파이프(VG1)와 엘보 몸통 내경 차이로 생기는 턱이 그대로 재현된다.
"""
import math
from dataclasses import dataclass

import numpy as np
import yaml


VIEW = np.array([-0.5, -0.25, 0.83])   # GUI 카메라 방향 (launch 의 follow offset 과 맞춤)


@dataclass
class Stave:
    center: np.ndarray   # 박스 중심 (world)
    rot: np.ndarray      # 3x3, 열 = 박스 x(배관 길이), y(원주 접선), z(반경 바깥)
    size: tuple          # (길이, 폭, 두께)
    inner_r: float       # 이 조각이 만드는 내벽 반경
    top: bool            # 위쪽 조각 (GUI에서 안을 보이게 시각만 생략)
    col_t: float = 0.0   # 충돌용 두께 (시각용 두께보다 두껍게: 얇으면 바퀴가 벽을 뚫고 나감)

    def col_center(self):
        return self.center + self.rot[:, 2] * (self.col_t - self.size[2]) / 2


def _axis_rot(axis, ang):
    """Rodrigues: axis(단위벡터) 기준 ang 회전 행렬."""
    a = np.asarray(axis, float)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * (k @ k)


def rot_to_rpy(r):
    """3x3 -> SDF roll pitch yaw (Z-Y-X)."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def load_config(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


class CourseBuilder:
    # 굽힘 방향 -> 진행 프레임 기준 회전축 (x=진행, y=왼쪽, z=위)
    DIRS = {'left': (0, 0, 1), 'right': (0, 0, -1), 'up': (0, -1, 0), 'down': (0, 1, 0)}

    def __init__(self, cfg, fitting_override=None):
        p = cfg['pipe']
        self.cfg = cfg
        self.pipe_r = p['inner_d'] / 2
        self.wall = p['wall']
        self.col_wall = max(p.get('collision_wall', 0.03), self.wall)
        self.n = int(p['staves'])
        self.slice = math.radians(p['slice_deg'])
        self.pos = np.array([0.0, 0.0, p['axis_height']])
        self.frame = np.eye(3)       # 열 = 진행(x), 왼쪽(y), 위(z)
        self.staves = []
        self.features = []           # (이름, 시작 거리[m]) — 중심선 기준 누적 거리
        self.s = 0.0
        self.override = fitting_override or {}
        self.path = [(self.pos.copy(), self.frame[:, 0].copy(), self.frame[:, 2].copy())]   # 중심선 (점, 접선, 위쪽) 5 mm 간격

    # ---- 박스 하나 ----
    def _add(self, mid, t, b, n_axis, r, length, room=None):
        """mid: 중심선 점, t: 진행 방향, b/n_axis: 원주 기준 직교축.
        room(phi): 벽 바깥쪽으로 쓸 수 있는 거리 (엘보 안쪽은 굽힘 중심축을 넘지 않게)."""
        width = 2 * r * math.tan(math.pi / self.n) * 1.02
        for k in range(self.n):
            phi = 2 * math.pi * k / self.n
            u = math.cos(phi) * b + math.sin(phi) * n_axis       # 반경 바깥 방향
            ln = length(phi) if callable(length) else length
            c = mid + u * (r + self.wall / 2)
            y = np.cross(u, t)
            rot = np.column_stack([t, y, u])
            col_t = self.col_wall if room is None else min(self.col_wall, max(self.wall, 0.9 * room(phi)))
            # 카메라(GUI가 로봇을 비스듬히 위·뒤·오른쪽에서 따라감) 쪽 벽은 시각만 생략 -> 수직관에서도 안이 보임
            self.staves.append(Stave(c, rot, (ln, width, self.wall), r, bool(np.dot(u, VIEW) > 0.4), col_t))

    def straight(self, length, r=None):
        r = self.pipe_r if r is None else r
        if length <= 0:
            return
        t, yl, zu = self.frame[:, 0], self.frame[:, 1], self.frame[:, 2]
        self._add(self.pos + t * length / 2, t, yl, zu, r, length)
        for k in range(1, int(length / 0.005) + 1):
            self.path.append((self.pos + t * min(k * 0.005, length), t.copy(), zu.copy()))
        self.pos = self.pos + t * length
        self.s += length

    def bend(self, angle_deg, R, r, direction):
        ang = math.radians(angle_deg)
        a_loc = np.array(self.DIRS[direction], float)
        a = self.frame @ a_loc                       # 회전축 (world)
        t0 = self.frame[:, 0]
        c_dir = np.cross(a, t0)                      # 굽힘 중심 쪽
        center = self.pos + R * c_dir
        m = max(1, int(math.ceil(ang / self.slice)))
        d = ang / m
        for i in range(m):
            rm = _axis_rot(a, (i + 0.5) * d)
            ti = rm @ t0
            outward = -(rm @ c_dir)                  # 굽힘 바깥쪽
            mid = center + R * outward
            # 원주 각 phi에서 굽힘축까지 거리 = R + r cos(phi) -> 그 위치의 조각 길이
            self._add(mid, ti, outward, a, r,
                      lambda phi: 2 * max(R + r * math.cos(phi), 1e-4) * math.tan(d / 2) + 3e-4,
                      room=lambda phi: R + r * math.cos(phi) if math.cos(phi) < 0 else 1.0)
        for k in range(1, int(R * ang / 0.005) + 1):
            rk = _axis_rot(a, min(k * 0.005 / R, ang))
            self.path.append((center - R * (rk @ c_dir), rk @ t0, rk @ self.frame[:, 2]))
        rm = _axis_rot(a, ang)
        self.pos = center - R * (rm @ c_dir)
        self.frame = rm @ self.frame
        self.s += R * ang

    def fitting(self, name, direction):
        f = self.cfg['fittings'][self.override.get(name, name)]
        r = f['bore'] / 2
        leg = max(f['Z'] - f['R'] * math.tan(math.radians(f['angle']) / 2), 0.0)
        self.features.append((f"{name}:{direction}", self.s))
        self.straight(leg, r)
        self.bend(f['angle'], f['R'], r, direction)
        self.straight(leg, r)

    def cap(self):
        t = self.frame[:, 0]
        rr = self.pipe_r + self.wall
        c = self.pos + t * self.wall / 2
        # 판의 z(두께)를 진행 방향으로. 충돌은 두껍게 (얇으면 기운 로봇이 밀고 나감)
        rot = np.column_stack([-self.frame[:, 2], self.frame[:, 1], t])
        self.staves.append(Stave(c, rot, (2 * rr, 2 * rr, self.wall), self.pipe_r, False, self.col_wall))
        self.features.append(('cap', self.s))

    def build(self, course):
        for seg in self.cfg['courses'][course]:
            if 'start_z' in seg:          # 시작 높이 (아래로 꺾이는 코스용)
                self.pos[2] = float(seg['start_z'])
                self.path = [(self.pos.copy(), self.frame[:, 0].copy(), self.frame[:, 2].copy())]
            elif 'straight' in seg:
                self.straight(float(seg['straight']))
            elif 'fitting' in seg:
                self.fitting(seg['fitting'], seg.get('dir', 'left'))
            elif seg.get('cap'):
                self.cap()
        return self


def build_course(cfg, course, fitting_override=None):
    return CourseBuilder(cfg, fitting_override).build(course)


# ---------- 출력 ----------
def _f(v):
    return ' '.join(f'{x:.6f}' for x in v)


def write_visual_stl(builder, path, cutaway=True):
    """모든 벽 조각의 시각용 박스를 STL 하나로 합침 (박스 수백 개를 따로 그리면 GUI가 매우 느림)."""
    import struct
    faces = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1),
             (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)]
    tris = []
    for st in builder.staves:
        if cutaway and st.top:
            continue
        h = np.array(st.size) / 2
        corners = [st.center + st.rot @ (h * np.array(sg)) for sg in
                   [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                    (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]]
        for a, b, c in faces:
            tris.append((corners[a], corners[b], corners[c]))
    with open(path, 'wb') as f:
        f.write(b'pipe'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))


def write_collision_stl(builder, path):
    """충돌용: 배관 내벽면만 메시 하나로 (법선은 배관 안쪽).
    박스 수백 개를 충돌체로 두면 DART가 매 스텝 박스끼리도 검사해서 실시간 대비 0.2배까지 느려짐."""
    import struct
    pts = inner_triangles(builder, cutaway=False)
    with open(path, 'wb') as f:
        f.write(b'pipe_collision'.ljust(80, b' '))
        f.write(struct.pack('<I', len(pts) // 3))
        for i in range(0, len(pts), 3):
            a, b, c = pts[i], pts[i + 2], pts[i + 1]          # 감는 방향을 뒤집어 법선이 안쪽
            n = np.cross(b - a, c - a)
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))


def world_sdf(builder, world_name='pipe_world', cutaway=True, visual_mesh=None, collision_mesh=None,
              sensor_mesh=None):
    """visual_mesh / collision_mesh: STL 경로. 주면 메시 하나로, 없으면 박스마다.
    sensor_mesh: 잘라내지 않은 배관 STL. ToF(gpu_lidar) 전용 (visibility 0x02 + 완전 투명이라 GUI 화면엔 안 보임),
                 주면 렌더링 센서용 Sensors 시스템도 켬."""
    links = []
    if sensor_mesh:
        links.append(
            f'<visual name="pipe_sensor_visual"><visibility_flags>0x02</visibility_flags><transparency>1.0</transparency>'
            f'<geometry><mesh><uri>{sensor_mesh}</uri></mesh></geometry>'
            '<material><ambient>1 1 1 0</ambient><diffuse>1 1 1 0</diffuse></material></visual>')
    if collision_mesh:
        links.append(
            f'<collision name="pipe_collision"><geometry><mesh><uri>{collision_mesh}</uri></mesh></geometry>'
            '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>')
    if visual_mesh:
        links.append(
            f'<visual name="pipe_visual"><geometry><mesh><uri>{visual_mesh}</uri></mesh></geometry>'
            '<material><ambient>0.35 0.5 0.65 1</ambient><diffuse>0.45 0.6 0.75 1</diffuse><specular>0.2 0.2 0.2 1</specular></material></visual>')
    for i, s in enumerate(builder.staves):
        rpy = list(rot_to_rpy(s.rot))
        pose = _f(list(s.center) + rpy)
        size = _f(s.size)
        cpose = _f(list(s.col_center()) + rpy)
        csize = _f((s.size[0], s.size[1], s.col_t))
        if not collision_mesh:
            links.append(
            f'<collision name="c{i}"><pose>{cpose}</pose><geometry><box><size>{csize}</size></box></geometry>'
            '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>')
        if not visual_mesh and not (cutaway and s.top):
            links.append(
                f'<visual name="v{i}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
                '<material><ambient>0.35 0.5 0.65 1</ambient><diffuse>0.45 0.6 0.75 1</diffuse><specular>0.2 0.2 0.2 1</specular></material></visual>')
    body = '\n        '.join(links)
    sensors_sys = ('\n    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">'
                   '<render_engine>ogre2</render_engine></plugin>') if sensor_mesh else ''
    return f'''<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{world_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>{sensors_sys}
    <gravity>0 0 -9.81</gravity>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.75 0.8 0.85 1</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.25 0.25 0.27 1</ambient><diffuse>0.3 0.3 0.32 1</diffuse></material></visual>
      </link>
    </model>
    <model name="pipe">
      <static>true</static>
      <link name="link">
        {body}
      </link>
    </model>
  </world>
</sdf>
'''


def inner_triangles(builder, cutaway=True):
    """RViz TRIANGLE_LIST용: 각 박스의 안쪽 면(직사각형 = 삼각형 2개)."""
    pts = []
    for s in builder.staves:
        if cutaway and s.top:
            continue
        x, y, z = s.rot[:, 0], s.rot[:, 1], s.rot[:, 2]
        c = s.center - z * s.size[2] / 2
        hx, hy = x * s.size[0] / 2, y * s.size[1] / 2
        p = [c - hx - hy, c + hx - hy, c + hx + hy, c - hx + hy]
        pts += [p[0], p[1], p[2], p[0], p[2], p[3]]
    return pts


def nearest_tangent(builder, p):
    """점 p에서 가장 가까운 중심선 점의 접선 (배관 진행 방향)."""
    pts = np.array([q[0] for q in builder.path])
    i = int(np.argmin(np.linalg.norm(pts - p, axis=1)))
    return builder.path[i][1]
