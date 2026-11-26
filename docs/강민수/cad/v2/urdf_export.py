"""앞 모듈 v2 CAD → 시뮬레이션 모델 front_v2 의 메시와 CAD 값 (simulation/src/pipe_robot_sim/models/front_v2/).

만드는 것: meshes/*.stl (보이는 모양), cad_params.xacro (치수, 스프링, 링크별 질량·무게중심·관성)
손으로 쓴 것: front_v2.urdf.xacro (링크·관절 구조), robot.yaml  → 이 스크립트가 덮어쓰지 않는다.

좌표: CAD (X, Y, Z) mm → URDF (x, y, z) m 는 x = Z (앞), y = X, z = Y (위). 유닛 0 의 바퀴가 위(+z).
질량: 출력 부품은 CAD 부피 × PLA 1.24 g/cm3 (속 100 % 가정), 봉·축·베어링·부싱은 단순 형상 × 강 7.85 g/cm3,
      모터 96 g · 바퀴 28 g 은 판매 표기 (모양 안에 고르게 있다고 봄). 나사·스프링·전선·보드는 넣지 않았다. 모두 추정값.
실행: python urdf_export.py   (cadquery 필요, 1분 안팎)
검사: python urdf_check.py    (xacro 필요)
"""
import math, struct, pathlib, datetime
import numpy as np
import cadquery as cq
import front_module_v2 as F

HERE = pathlib.Path(__file__).resolve().parent
MODEL = HERE.parents[3] / "simulation" / "src" / "pipe_robot_sim" / "models" / "front_v2"
RHO_PLA, RHO_STEEL = 1.24e-3, 7.85e-3              # g/mm3
M_MOTOR, M_WHEEL = 96.0, 28.0                      # g [판매]: FIT0522, Nexus 14166
C202_FREE, C202_K = 16.0, 1.41                     # 자유 길이 mm [판매], 상수 N/mm [계산: (9.76 - 6.08) / (11.69 - 9.09)]
P = np.array([[0, 0, 1.0], [1, 0, 0], [0, 1, 0]])  # CAD → URDF 축 바꿈

def tris(wp, tol=0.1, ang=0.3):
    out = []
    for s in wp.solids().vals():
        v, t = s.tessellate(tol, ang)
        if t: out.append(np.array([(p.x, p.y, p.z) for p in v])[np.array(t)])
    return np.concatenate(out)
def write_stl(name, parts, shift=(0, 0, 0)):
    """parts 의 삼각형을 URDF 좌표(m)로 바꿔 바이너리 STL 로 저장."""
    T = (np.concatenate([tris(p) for p in parts]) + np.array(shift)) @ P.T / 1000.0
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-30
    f = MODEL / "meshes" / f"{name}.stl"
    with open(f, "wb") as fh:
        fh.write(b"front_v2 mesh (m, URDF axes) from urdf_export.py".ljust(80, b" ")); fh.write(struct.pack("<I", len(T)))
        for nn, t in zip(n.astype("<f4"), T.astype("<f4")): fh.write(nn.tobytes() + t.tobytes() + b"\0\0")
    return len(T), f.stat().st_size

def props(wp, rho=None, mass=None):
    """(질량 g, 무게중심 mm, 무게중심 기준 관성 g·mm2). mass 를 주면 모양 안에 고르게 있다고 본다."""
    M, C, I = 0.0, np.zeros(3), []
    for s in wp.solids().vals():
        v = s.Volume(); c = cq.Shape.centerOfMass(s); c = np.array([c.x, c.y, c.z])
        m = cq.Shape.matrixOfInertia(s); I.append((v, c, np.array(m)))        # 밀도 1, 무게중심 기준
        M += v; C += v * c
    C /= M; J = np.zeros((3, 3))
    for v, c, i in I:
        r = c - C; J += i + v * (r @ r * np.eye(3) - np.outer(r, r))
    k = (mass / M) if mass is not None else rho
    return M * k, C, J * k
def combine(items, shift=(0, 0, 0)):
    M = sum(m for m, _, _ in items); C = sum(m * c for m, c, _ in items) / M; J = np.zeros((3, 3))
    for m, c, i in items:
        r = c - C; J += i + m * (r @ r * np.eye(3) - np.outer(r, r))
    return M / 1000.0, P @ (C + np.array(shift)) / 1000.0, P @ J @ P.T * 1e-9          # kg, m, kg·m2 (URDF 축)
def inertial_macro(name, mci):
    m, c, i = mci
    return (f'  <xacro:macro name="cad_inertial_{name}">\n    <inertial>\n      <origin xyz="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" rpy="0 0 0"/>\n'
            f'      <mass value="{m:.5f}"/>\n      <inertia ixx="{i[0,0]:.4e}" ixy="{i[0,1]:.4e}" ixz="{i[0,2]:.4e}" iyy="{i[1,1]:.4e}" iyz="{i[1,2]:.4e}" izz="{i[2,2]:.4e}"/>\n'
            f'    </inertial>\n  </xacro:macro>\n')

if __name__ == "__main__":
    t = props(F.box(0, 10, 0, 20, 0, 30), rho=1.0)                              # 관성 함수 확인: 상자 10 × 20 × 30
    assert abs(t[2][0, 0] - 6000 * (400 + 900) / 12) < 1 and np.allclose(t[1], (5, 10, 15)), t
    (MODEL / "meshes").mkdir(parents=True, exist_ok=True)
    mv = F.bought_moving(); r3 = lambda f: [F.rotz(f, 120 * k) for k in range(3)]
    base, cap, cover, slider, hub = F.base(), F.front_cap(), F.cover(), F.slider(), F.hub()
    assert F.COVER_INFO["path_overlap_mm3"] == 0, F.COVER_INFO
    rods = r3(F.rods_unit())
    spacers = [F.rotz(F.spacer().translate((0, 0, gz - F.GUIDE_Z[0])), 120 * k) for k in range(3) for gz in F.GUIDE_Z]
    info = dict(body=write_stl("body", [base, cap]), rods=write_stl("rods", rods), spacers=write_stl("spacers", spacers),
                cover=write_stl("cover", [cover]), slider=write_stl("slider", [slider]),
                motor=write_stl("motor", [mv["motor"], mv["plug"], mv["worm"], mv["bearing"], mv["bushing"]]),
                wheel=write_stl("wheel", [mv["wheel"]], shift=(0, -F.A0, 0)),
                axle=write_stl("axle", [mv["axle"], hub, mv["wormwheel"]], shift=(0, -F.A0, 0)))
    body = combine([props(base, RHO_PLA), props(cap, RHO_PLA), props(cover, RHO_PLA)] + [props(r, RHO_STEEL) for r in rods])
    unit = combine([props(slider, RHO_PLA), props(mv["motor"], mass=M_MOTOR), props(F.worm(), RHO_PLA),
                    props(mv["bearing"], RHO_STEEL), props(mv["bushing"], RHO_STEEL)])
    wheel = combine([props(mv["wheel"], mass=M_WHEEL), props(hub, RHO_PLA), props(F.wormwheel(), RHO_PLA), props(mv["axle"], RHO_STEEL)],
                    shift=(0, -F.A0, 0))
    total = body[0] + 3 * (unit[0] + wheel[0])
    mm = lambda v: f"{v / 1000:.5f}"
    pr = [("cad_wheel_r", mm(F.WR), "바퀴 반지름 (Ø38.1) [도면]"), ("cad_wheel_w", mm(F.WW), "바퀴 폭 [도면]"),
          ("cad_axle_h", mm(F.A0), "변위 0 일 때 바퀴 축 높이 (내경 120 접촉)"),
          ("cad_d_min", mm(F.D_MIN), "기구 행정 아래 끝"), ("cad_d_max", mm(F.D_MAX), "기구 행정 위 끝"),
          ("cad_spring_k", f"{2 * C202_K * 1000:.0f}", "바퀴당 스프링 상수 N/m (C202 2개) [계산]"),
          ("cad_spring_gap0", mm(F.E0 - F.SEAT_Y), "변위 0, 스페이서 없을 때 스프링 길이 (받침 ~ 귀 아랫면)"),
          ("cad_spring_free", mm(C202_FREE), "C202 자유 길이 [판매]"), ("cad_spring_min", mm(F.C202_LMIN), "C202 최소 길이 [판매]"),
          ("cad_spacer", mm(F.SPACER), "스페이서 (내경 120 설정)"),
          ("cad_x_rear", mm(F.Z_REAR), "뒷판 뒷면"), ("cad_x_front", mm(F.Z_FRONT), "앞판 앞면"), ("cad_plate_r", mm(F.PLATE_R), "앞·뒷판 반지름"),
          ("cad_tri_d", mm(F.TRI_D), "삼각 외피 평면까지 거리"), ("cad_tri_rv", mm(F.TRI_RV), "삼각 외피 꼭짓점 반지름"),
          ("cad_tri_vtx", f"{math.radians(F.TRI_VTX):.5f}", "꼭짓점 방향 rad (+120° 간격)"),
          ("cad_rod_top", mm(F.ROD_TOP), "가이드봉 끝 반지름 (긴 봉)"), ("cad_rod_top_short", mm(F.ROD_TOP - 10), "가이드봉 끝 반지름 (짧은 봉)"),
          ("cad_rod_x0", mm(F.GUIDE_Z[0]), "가이드봉 위치 (앞)"), ("cad_rod_x1", mm(F.GUIDE_Z[1]), "가이드봉 위치 (뒤)"),
          ("cad_motor_y", mm(F.MX), "모터 축 옆 오프셋"), ("cad_motor_z", mm(F.MY), "모터 축 높이 (변위 0)"), ("cad_motor_r", mm(12.5), "모터 반지름 [도면]"),
          ("cad_motor_x0", mm(F.Z_ENC), "모터 뒤끝 (엔코더)"), ("cad_motor_x1", mm(F.M_FACE), "모터 장착면"),
          ("cad_gear_ratio", str(F.WHEEL_Z), "웜 감속비 1:n"),
          ("cad_mass_total", f"{total:.4f}", "몸체 + 유닛 3 + 바퀴 3 (kg, 추정)")]
    x = ['<?xml version="1.0"?>', f"<!-- 자동 생성: docs/강민수/cad/v2/urdf_export.py ({datetime.date.today()}). 직접 고치지 말고 CAD 를 고친 뒤 다시 만들 것.",
         "     단위 m, kg, rad. 좌표는 URDF (x 앞, z 위 = 유닛 0 바퀴 쪽). 질량·관성은 추정 (출력 부품 속 100 %, 나사·스프링·전선·보드 제외).",
         f"     몸체 {body[0]*1000:.0f} g, 유닛(슬라이더 + 모터) {unit[0]*1000:.0f} g × 3, 바퀴 축 뭉치 {wheel[0]*1000:.0f} g × 3, 합 {total*1000:.0f} g -->",
         '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">']
    x += [f'  <xacro:property name="{n}" value="{v}"/>   <!-- {c} -->' for n, v, c in pr]
    x += [inertial_macro("body", body), inertial_macro("unit", unit), inertial_macro("wheel", wheel), "</robot>", ""]
    (MODEL / "cad_params.xacro").write_text("\n".join(x), encoding="utf-8")
    for n, (nt, sz) in info.items(): print(f"meshes/{n}.stl: 삼각형 {nt}, {sz/1024:.0f} kB")
    for n, (m, c, i) in (("몸체", body), ("유닛", unit), ("바퀴 축 뭉치", wheel)):
        print(f"{n}: {m*1000:.1f} g, 무게중심 (x, y, z) = ({c[0]*1000:.1f}, {c[1]*1000:.1f}, {c[2]*1000:.1f}) mm, 관성 대각 {np.diag(i)}")
    print(f"합 {total*1000:.0f} g → {MODEL / 'cad_params.xacro'}")
