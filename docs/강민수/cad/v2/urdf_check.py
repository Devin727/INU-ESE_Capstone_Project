"""시뮬레이션 모델 front_v2 검사 (Gazebo·ROS 없이). xacro 를 URDF 로 풀어 구조와 값을 확인하고 미리보기 그림을 그린다.
확인하는 것: 문법, 링크·관절 연결, 메시 파일, 관성 값, docs/02 규약의 플러그인·관절 이름, 관절 0 에서 바퀴와 벽의 틈,
            바퀴 회전 방향, 스프링 힘, 충돌체가 배관 안에 드는지.
확인하지 않는 것: Gazebo 안에서의 움직임 (접촉, 마찰, 스프링 관절, 주행). 이 스크립트는 물리 시뮬을 돌리지 않는다.
실행: python -X utf8 urdf_check.py   (xacro, pyyaml, numpy, matplotlib 필요. -X utf8 은 Windows 에서 xacro 가 한글 주석을 읽게 함)
출력: urdf_check_results.txt, ../../figures/2026-11-26-앞모듈_v2_URDF_미리보기.png
"""
import math, pathlib, xml.etree.ElementTree as ET
import numpy as np, yaml, xacro
import xacro.substitution_args as SA
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = pathlib.Path(__file__).resolve().parent
PKG = HERE.parents[3] / "simulation" / "src" / "pipe_robot_sim"
MODEL = PKG / "models" / "front_v2"
SA._eval_find = lambda pkg: PKG.as_posix()          # $(find pipe_robot_sim) → 소스 폴더 (ament 없이)
for f in ("C:/Windows/Fonts/malgun.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
    try: fm.fontManager.addfont(f); plt.rcParams["font.family"] = fm.FontProperties(fname=f).get_name(); break
    except Exception: pass
plt.rcParams["axes.unicode_minus"] = False

def vec(s): return np.array([float(v) for v in s.split()])
def rot(rpy):
    r, p, y = rpy; cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
def tf(el):
    T = np.eye(4); o = el.find("origin") if el is not None else None
    if o is not None: T[:3, :3] = rot(vec(o.get("rpy", "0 0 0"))); T[:3, 3] = vec(o.get("xyz", "0 0 0"))
    return T
def read_stl(path):
    d = np.fromfile(path, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), offset=84); return d["v"].astype(float)

class Model:
    def __init__(self, pipe_d, **args):
        self.pipe_d = pipe_d
        doc = xacro.process_file(str(MODEL / "front_v2.urdf.xacro"), mappings=dict(pipe_inner_d=str(pipe_d), params_file=str(MODEL / "robot.yaml"), suspension="all", **args))
        self.xml = doc.toxml(); self.root = ET.fromstring(self.xml)
        self.links = {l.get("name"): l for l in self.root.findall("link")}; self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.mat = {m.get("name"): vec(m.find("color").get("rgba")) for m in self.root.findall("material")}
        child = {j.find("child").get("link"): j for j in self.joints.values()}
        self.rootlink = [n for n in self.links if n not in child]
        self.T = {}
        def fk(n):
            if n not in self.T:
                j = child.get(n); self.T[n] = np.eye(4) if j is None else fk(j.find("parent").get("link")) @ tf(j)
            return self.T[n]
        for n in self.links: fk(n)
    def inertial(self, n):
        i = self.links[n].find("inertial")
        if i is None: return None
        a = i.find("inertia"); g = lambda k: float(a.get(k))
        return float(i.find("mass").get("value")), tf(i)[:3, 3], np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]])
    def visuals(self):
        for n, l in self.links.items():
            for v in l.findall("visual"):
                m = v.find("geometry/mesh")
                if m is None: continue
                tri = read_stl(m.get("filename").replace("file://", "")); T = self.T[n] @ tf(v)
                yield n, tri @ T[:3, :3].T + T[:3, 3], self.mat[v.find("material").get("name")]

def check(M, rp, log):
    Rp = M.pipe_d / 2; ok = True
    def want(cond, msg):
        nonlocal ok; ok &= bool(cond); log(("  확인 " if cond else "  문제 ") + msg)
    log(f"== 배관 내경 {M.pipe_d*1000:.1f} mm ==")
    want(M.rootlink == ["base_link"], f"루트 링크 {M.rootlink}, 링크 {len(M.links)}개, 관절 {len(M.joints)}개")
    miss = [m.get("filename") for m in M.root.iter("mesh") if not pathlib.Path(m.get("filename").replace("file://", "")).is_file()]
    want(not miss, f"메시 파일 {len(list(M.root.iter('mesh')))}개 모두 있음" if not miss else f"없는 메시 {miss}")
    total = 0.0; bad = []
    for n in M.links:
        I = M.inertial(n)
        if I is None: continue
        m, c, J = I; total += m; e = np.linalg.eigvalsh(J)
        if not (m > 0 and e[0] > 0 and e[0] + e[1] >= e[2] * 0.999): bad.append(n)
    want(not bad, f"관성 값 정상 (양수, 삼각 부등식). 질량 합 {total*1000:.0f} g [추정]" if not bad else f"관성 이상 {bad}")
    plug = [(p.get("name"), (p.findtext("joint_name") or "")) for g in M.root.findall("gazebo") for p in g.findall("plugin")]
    force = {j for n, j in plug if n.endswith("ApplyJointForce")}
    wj = rp["wheel_joints"]
    want(all(j in M.joints and M.joints[j].get("type") == "continuous" and j in force for j in wj), f"robot.yaml wheel_joints {wj}: 연속 관절 + ApplyJointForce")
    want(any(n.endswith("JointStatePublisher") for n, _ in plug) and any(n.endswith("OdometryPublisher") for n, _ in plug)
         and any(s.get("type") == "imu" for s in M.root.iter("sensor")), "JointStatePublisher, OdometryPublisher, IMU 센서 있음")
    stall, nl = float(rp["motor_stall_torque"]), float(rp["motor_no_load_speed"])
    for k in range(3):
        sj, wjn = M.joints[f"u{k}_slide_joint"], M.joints[f"u{k}_wheel_joint"]; Tw = M.T[f"u{k}_wheel"]; c = Tw[:3, 3]
        rad = c / np.linalg.norm(c); axis = Tw[:3, :3] @ vec(wjn.find("axis").get("xyz"))
        col = M.links[f"u{k}_wheel"].find("collision/geometry")
        if col.find("cylinder") is not None:
            r, w = float(col.find("cylinder").get("radius")), float(col.find("cylinder").get("length")); reach = math.hypot(np.linalg.norm(c) + r, w / 2)
        else: r = float(col.find("sphere").get("radius")); reach = np.linalg.norm(c) + r
        fwd = -np.cross(axis, rad * r)                                 # + 회전일 때 벽이 몸체를 미는 방향
        lim = sj.find("limit"); lo, hi = float(lim.get("lower")), float(lim.get("upper"))
        sp = [g for g in M.root.findall("gazebo") if g.get("reference") == f"u{k}_slide_joint"][0]
        ks, ref = float(sp.findtext("springStiffness")), float(sp.findtext("springReference")); gap = Rp - reach
        f_wall = ks * (ref - gap)                                      # 바퀴가 벽에 닿았을 때 (관절값 = gap)
        want(abs(c[0]) < 1e-9 and abs(math.degrees(math.atan2(-rad[1], rad[2])) % 360 - 120 * k) < 1e-6, f"유닛 {k}: 바퀴 중심 반지름 {np.linalg.norm(c)*1000:.2f} mm, 방향 위에서 {120*k}°")
        want(0 < gap < 0.001, f"유닛 {k}: 관절 0 에서 바퀴와 벽의 틈 {gap*1000:.2f} mm")
        want(fwd[0] > 0.99 * r and abs(axis @ rad) < 1e-9, f"유닛 {k}: + 회전에 앞(+x)으로 감, 바퀴 축은 접선 방향")
        want(lo < gap < hi and f_wall > 0, f"유닛 {k}: 관절 범위 {lo*1000:.2f} ~ {hi*1000:.2f} mm, 스프링 {ks:.0f} N/m, 벽에 닿았을 때 누르는 힘 {f_wall:.2f} N [계산]")
        dmp = float(wjn.find("dynamics").get("damping"))
        want(abs(dmp - stall / nl) < 1e-6 * dmp, f"유닛 {k}: 바퀴 관절 감쇠 {dmp:.3f} = robot.yaml 스톨 {stall} / 무부하 {nl}")
    worst = 0.0
    for n, l in M.links.items():                                       # 바퀴 말고 다른 충돌체가 배관 반지름 안에 드는지 (직관, 관절 0)
        if n.endswith("_wheel") or n.startswith("rear_skid"): continue
        for cc in l.findall("collision"):
            cy = cc.find("geometry/cylinder"); T = M.T[n] @ tf(cc); r, L = float(cy.get("radius")), float(cy.get("length")); ax = T[:3, 2]
            ends = [T[:3, 3] + s * L / 2 * ax for s in (-1, 1)]
            ext = math.hypot(T[1, 3], T[2, 3]) + r if abs(ax[0]) > 0.99 else max(math.hypot(math.hypot(e[1], e[2]), r) for e in ends)   # 축 방향 원통 / 반지름 방향 원통(끝 모서리)
            worst = max(worst, ext)
    want(worst < Rp, f"바퀴 아닌 충돌체의 최대 반지름 {worst*1000:.1f} mm < 배관 반지름 {Rp*1000:.1f} mm (틈 {(Rp-worst)*1000:.1f} mm)")
    skids = [n for n in M.links if n.startswith("rear_skid")]
    if skids:                                                          # 임시 뒤 받침 (rear_support:=true)
        gaps = []; xs = []
        for n in skids:
            c = M.T[n][:3, 3]; r = float(M.links[n].find("collision/geometry/sphere").get("radius")); gaps.append(Rp - (math.hypot(c[1], c[2]) + r)); xs.append(c[0])
        ang = sorted(round(math.degrees(math.atan2(M.T[n][2, 3], M.T[n][1, 3])) % 360) for n in skids)
        want(len(skids) == 3 and all(0 < g < 0.003 for g in gaps) and ang[1] - ang[0] == 120 and ang[2] - ang[1] == 120,
             f"임시 뒤 받침 3개: 벽과의 틈 {min(gaps)*1000:.2f} mm, x {xs[0]*1000:.1f} mm, 방향 {ang}° (y 축에서 z 쪽으로). 허용 기울기 약 {math.degrees(math.atan(max(gaps) / abs(xs[0]))):.2f}° [계산]")
    return ok

def end_view(ax, M, title):
    P = [t[:, :, [1, 2]] * 1000 for _, t, _ in M.visuals()]; C = [np.r_[c[:3], min(c[3], 1) * 0.35] for _, t, c in M.visuals() for _ in t]
    ax.add_collection(PolyCollection(np.concatenate(P), facecolors=C, edgecolors="none"))
    th = np.linspace(0, 2 * np.pi, 361); R = M.pipe_d * 500; ax.plot(R * np.cos(th), R * np.sin(th), "k", lw=1.5)
    for n, l in M.links.items():                                       # 충돌체 (빨간 선)
        for cc in l.findall("collision"):
            T = M.T[n] @ tf(cc); g = cc.find("geometry"); c = T[:3, 3] * 1000
            if g.find("sphere") is not None: r = float(g.find("sphere").get("radius")) * 1000; ax.plot(c[1] + r * np.cos(th), c[2] + r * np.sin(th), "r", lw=.9)
            else:
                r, L = float(g.find("cylinder").get("radius")) * 1000, float(g.find("cylinder").get("length")) * 1000; a = T[:3, 2]
                if abs(a[0]) > 0.99: ax.plot(c[1] + r * np.cos(th), c[2] + r * np.sin(th), "r", lw=.9)
                else:
                    u = a[1:] / np.linalg.norm(a[1:]); v = np.array([-u[1], u[0]]); q = [c[1:] + s * L / 2 * u + t * r * v for s, t in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))]
                    ax.plot(*np.array(q).T, "r", lw=.9)
    ax.set_aspect("equal"); ax.set_xlim(-66, 66); ax.set_ylim(-66, 66); ax.axis("off"); ax.set_title(title, fontsize=13, loc="left")
    ax.annotate("", xy=(0, 64), xytext=(0, 52), arrowprops=dict(arrowstyle="->", lw=1)); ax.text(2, 61, "z (위)", fontsize=10); ax.text(46, -64, "y →", fontsize=10)

def view3d(ax, M, title):
    light = np.array([0.35, -0.55, 0.75]); light /= np.linalg.norm(light); TT, CC = [], []
    for _, t, c in M.visuals():
        t = t * 1000; n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
        TT.append(t); CC.append(np.c_[np.clip(c[None, :3] * (0.45 + 0.55 * np.abs(n @ light))[:, None], 0, 1), np.full(len(t), c[3])])
    TT = np.concatenate(TT); CC = np.concatenate(CC); ax.add_collection3d(Poly3DCollection(TT, facecolors=CC, edgecolors="none"))
    lo, hi = TT.reshape(-1, 3).min(0), TT.reshape(-1, 3).max(0)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2]); ax.set_box_aspect(hi - lo); ax.view_init(elev=20, azim=-55); ax.set_axis_off()
    ax.set_title(title, fontsize=13, loc="left")

if __name__ == "__main__":
    rp = yaml.safe_load((MODEL / "robot.yaml").read_text(encoding="utf-8"))["/**"]["ros__parameters"]
    lines = []
    def log(s): print(s, flush=True); lines.append(s)
    log("front_v2 모델 검사: xacro 를 URDF 로 풀어 값 확인 (Gazebo 실행 아님)")
    M100, M120 = Model(0.1008), Model(0.120); Msph = Model(0.120, wheel_shape="sphere")
    res = [check(M100, rp, log), check(M120, rp, log)]
    log("-- 아래는 wheel_shape:=sphere (바퀴 충돌체를 구로) --"); res.append(check(Msph, rp, log))
    for d in (0.1008, 0.120):
        log("-- 아래는 rear_support:=true (시뮬 전용 임시 뒤 받침) --"); res.append(check(Model(d, rear_support="true"), rp, log))
    log("결과: " + ("모두 확인" if all(res) else "문제 있음"))
    (HERE / "urdf_check_results.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    fig = plt.figure(figsize=(19, 7.4), dpi=100)
    fig.suptitle("앞 모듈 v2 시뮬레이션 모델 front_v2 미리보기 (URDF 를 풀어 그린 것, Gazebo 실행 화면 아님)", fontsize=16, fontweight="bold", y=.98)
    view3d(fig.add_axes([.0, .02, .36, .86], projection="3d"), M120, "① URDF 의 보이는 모양 (내경 120 설정)")
    end_view(fig.add_axes([.37, .06, .30, .82]), M100, "② 앞에서 본 모습, 시뮬 기본 배관 내경 100.8 (스페이서 없음)")
    end_view(fig.add_axes([.69, .06, .30, .82]), M120, "③ 내경 120 (스페이서 9, 긴 봉)")
    fig.text(.5, .015, "검은 원 = 배관 안쪽 면, 빨간 선 = 충돌체 (바퀴는 원통, 몸체는 원통 4개 근사, 모터, 긴 가이드봉). 관절 0 = 바퀴가 벽에서 0.3 mm 떨어진 자리.", ha="center", fontsize=11)
    out = HERE.parents[1] / "figures" / "2026-11-26-앞모듈_v2_URDF_미리보기.png"; fig.savefig(out); log(f"그림: {out.name}")
