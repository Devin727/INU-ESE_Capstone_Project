"""앞 모듈 v2 CAD — 내경 120 시연 기준(C202 + 스페이서 9), 내경 100~120 호환 구조. 단위 mm.
덮개: 2026-11-24부터 삼각 외피(cover). 이전 원통 덮개는 cover_round 로 남겨 두고 STL·STEP만 함께 낸다.

좌표: Z = 배관 길이 방향(앞이 +). 유닛 좌표 x = 접선, y = 반지름. 유닛 k 는 Z 축으로 120k 도 회전.
배치 근거: ../../2026-11-20-앞모듈_v2_호환_검토.md, 그림: ../../figures/2026-11-21-앞모듈_배치도_v2.png
실행: python front_module_v2.py            → stl/, step/, check_results.txt (cadquery 필요, 수 분)
      python front_module_v2.py --no-check → 출력만
"""
import sys, math, itertools, json, pathlib
import cadquery as cq

OUT = pathlib.Path(__file__).resolve().parent

# ---------------- 치수 ----------------
R_PIPE = 60.0; WR = 19.05; WW = 23.0; A0 = R_PIPE - WR        # 바퀴 축 높이 40.95 (내경 120 접촉)
GEAR_M = 1.0; WORM_D1 = 12.0; WHEEL_Z = 16                    # 출력 웜기어: 모듈 1.0, 한 줄 웜(피치원 Ø12), 웜휠 16T → 1:16
WORM_L = 10.0; WHEEL_B = 5.5; GEAR_BL = 0.10                  # 웜 길이, 웜휠 폭, 이 두께를 줄여 만든 백래시
WORM_RA = WORM_D1 / 2 + GEAR_M; WHEEL_RA = GEAR_M * WHEEL_Z / 2 + GEAR_M   # 이끝 반지름 7, 9
WHEEL_PHASE = 17.5                                            # 맞물림 위상(deg): 이 각에서 웜과 겹침 0 [계산]
CD = (WORM_D1 + GEAR_M * WHEEL_Z) / 2; MX = 25.0              # 웜 축간 14, 모터 옆 오프셋
MY = A0 - CD                                                  # 모터 축 높이 26.95
M_FACE = -9.5                                                 # FIT0522 장착면. 웜(z -5~5)을 D축(장착면 +4.5~14.5)에 직접
Z_GB, Z_CAN, Z_ENC = M_FACE - 23, M_FACE - 54, M_FACE - 63    # FIT0522 [도면]
PLUG = 10.0                                                   # 엔코더 뒤 플러그·전선 공간 [가정]
Z_FRONT = 23.0; FC_T = 3.0                                    # 앞판 z 20~23 (바퀴 앞끝 19.05와 0.95 띄움)
Z_REAR = Z_ENC - PLUG - 3.0                                   # 뒷판 뒷면 -85.5
PLATE_R = 37.0; COV_RI, COV_RO = 37.0, 39.0                   # 앞·뒷판 Ø74, 이전 원통 덮개 Ø78 (두께 2)
GUIDE_Z = (-26.0, -41.0); SEAT_Y = 9.0; SPACER = 9.0          # 가이드봉 z, 스프링 받침, 스페이서(내경 120 설정)
ROD_Y0, ROD_TOP = 4.5, 47.0                                   # 가이드봉 Ø4 반지름 4.5~47 (길이 42.5), 끝 스토퍼 2
EAR_OFF, EAR_H = 10.95, 12.0                                  # 귀 아랫면 = 축 높이 - 10.95, 귀 높이 = LM4UU 길이 12
EAR_W = 6.0                                                   # 귀 반폭 (구멍 반지름 4.05 + 벽 1.95)
E0 = A0 - EAR_OFF                                             # 설치 상태 귀 아랫면 30
BRG_R, BUSH_R = 3.55, 4.05                                    # MF74ZZ Ø7 구멍, 리니어 볼부시 LM4UU(4×8×12 [판매]) Ø8 구멍
M_SCREW = (16.0, 18.0)                                        # 모터 M3 2개 간격 범위 → 긴 구멍 [미확인]
C202_LMIN = 9.09
D_SOLID = (C202_LMIN + EAR_OFF + SEAT_Y + SPACER) - A0        # 스프링 한계 변위 -2.91
D_MIN, D_MAX = -12.0, 3.0                                     # 기구 행정 (내경 100 -2 ~ 이음부 +1)
D_JOINT = 61.9 - WR - A0                                      # 내경 123.8 이음부 1.9
D_CHECK = (D_MIN, -10.0, D_SOLID, 0.0, D_JOINT, D_MAX)
AXLE_X1 = 31.6                                                # 바퀴 축 끝 (x -19.5 ~ 31.6, 길이 약 51)
COV_RI_IN = 35.0                                              # 원통 덮개 안쪽 보강: 부품이 지나지 않는 곳은 반지름 35까지 채움 (두께 4)
TRI_PSI = 72.0                                                # 삼각 외피 평면의 법선 각 (deg). 유닛 0 바퀴 방향 = 90°, 그 모터 쪽 = 0° 방향
TRI_VTX = TRI_PSI - 60                                        # 꼭짓점 방향 12° = 유닛 사이 빈 자리 (레일이 서는 곳)
TRI_D, TRI_RV, TRI_T = 39.5, 47.0, 3.0                        # 평면까지 거리, 꼭짓점 둥근 부분 반지름, 벽 두께
TRI_EDGE = 1.5; WIN_R = 3.0                                   # 레일의 모터 쪽 가장자리 최소 두께, 창 모서리 둥글림
WIN_X0, WIN_X1 = -19.5 - 1.0, MX + 14.75 + 1.0                # 창 폭: 허브·축 끝 ~ 모터 받침(반지름 14.75), 여유 1씩 (-20.5 ~ 40.75)
WIN_ZR = Z_ENC - PLUG - 1.0                                   # 창 뒤끝 = 플러그 끝 + 여유 1 (-83.5)
WIN_ZE = GUIDE_Z[1] - EAR_W - 1.0                             # 가운데 창 뒤끝 = 뒤쪽 귀 + 여유 1 (-48)
WIN_ZG = 11.0                                                 # 가운데 창 앞끝 = 웜휠·허브(반지름 9) + 여유 1 + 모서리 둥글림 몫 1
WIN_XE = -EAR_W - 1.0 - 0.9                                   # 귀 구간 창의 허브 쪽 끝 = 귀 + 여유 1 + 모서리 둥글림 몫 0.9 (-7.9)
WIN_XG = MX + WORM_RA + 1.0                                   # 웜기어 구간 창의 모터 쪽 끝 = 웜 이끝 + 여유 1 (33)
WIN_ZW = -(WR + 1.0) - 0.95                                   # 바퀴·허브 구간 창 뒤끝 = 바퀴 뒤끝 + 여유 (-21)
WIN_ZM = -6.5 + 1.0                                           # 모터 구간 창 앞끝 = 모터 고정판 앞면 + 여유 1 (-5.5)
JOINT_ANG = 79.0                                              # 관절 연결 나사·앞판 나사 각 (유닛 사이, deg)
SCREW_ANG = (-25.0, 45.0)                                     # 덮개 고정 M2 각 (유닛마다 앞뒤 2개씩)

# ---------------- 기본 형상 ----------------
def box(x0, x1, y0, y1, z0, z1): return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cylz(r, z0, z1, x=0, y=0): return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((x, y, z0))
def cylx(r, x0, x1, y, z): return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, y, z))
def cyly(r, y0, y1, x, z): return cylz(r, 0, y1 - y0).rotate((0, 0, 0), (1, 0, 0), -90).translate((x, y0, z))
def rotz(s, deg): return s.rotate((0, 0, 0), (0, 0, 1), deg)
def tube(ro, ri, z0, z1): return cylz(ro, z0, z1).cut(cylz(ri, z0 - 1, z1 + 1))
def radial(r, y0, y1, ang, z): return rotz(cyly(r, y0, y1, 0, z), -ang)       # 유닛 각 ang 방향 반지름 구멍
def prim(p):
    t = p[0]
    if t == "box": return box(*p[1:])
    if t == "cylz": return cylz(*p[1:])
    if t == "cylx": return cylx(*p[1:])
    if t == "cyly": return cyly(*p[1:])
def swept(p, d0, d1, c):
    """y 방향으로 d0~d1 움직일 때 지나는 공간 + 여유 c."""
    t = p[0]
    if t == "box":
        _, x0, x1, y0, y1, z0, z1 = p; return box(x0 - c, x1 + c, y0 + d0 - c, y1 + d1 + c, z0 - c, z1 + c)
    if t == "cylz":
        _, r, z0, z1, x, y = p; r += c
        return cylz(r, z0 - c, z1 + c, x, y + d0).union(cylz(r, z0 - c, z1 + c, x, y + d1)).union(box(x - r, x + r, y + d0, y + d1, z0 - c, z1 + c))
    if t == "cylx":
        _, r, x0, x1, y, z = p; r += c
        return cylx(r, x0 - c, x1 + c, y + d0, z).union(cylx(r, x0 - c, x1 + c, y + d1, z)).union(box(x0 - c, x1 + c, y + d0, y + d1, z - r, z + r))
    if t == "cyly":
        _, r, y0, y1, x, z = p; return cyly(r + c, y0 + d0 - c, y1 + d1 + c, x, z)

# ---------------- 슬라이더 (유닛마다 1개, 출력) ----------------
def slider_prims():
    p = [("box", 13, 21.5, A0 - 5, A0 + 5.05, -9.5, 6),          # 지지판 B: 바퀴 쪽 베어링
         ("box", 28, 30.5, A0 - 5, A0 + 5.05, -5.05, 5.05),       # 지지판 C: 웜휠 바깥 베어링 (베어링 간격 15)
         ("box", 28, 30.5, MY + 13, A0 + 5.05, -9.4, -5.05),      # 지지판 C ↔ 모터 위 막대
         ("box", 13, 21.5, MY + 4.9, A0 - 4.9, -9.5, -6.5),       # 지지판 B ↔ 모터 고정판
         ("box", MX - 12.6, MX + 12.6, MY - 5, MY + 5, -9.5, -6.5),   # 모터 고정판 (나사 줄 ±5 띠)
         ("box", 17, 33, MY + 13, MY + 16, -44, -9.3),            # 모터 위 막대
         ("cylz", 14.75, -29, -21, MX, MY), ("cylz", 14.75, -44, -36, MX, MY)]   # 모터 받침 (위 반원)
    for gz in GUIDE_Z: p.append(("box", -EAR_W, 14, E0, E0 + EAR_H, gz - EAR_W, gz + EAR_W))  # 귀(LM4UU) + 팔
    return p
def slider():
    P = slider_prims()
    s = prim(P[0]).union(prim(P[1])).union(prim(P[2])).union(prim(P[3])).union(prim(P[4]).intersect(cylz(12.6, -10, -6, MX, MY))).union(prim(P[5]))
    for z0, z1 in ((-29, -21), (-44, -36)):
        s = s.union(cylz(14.75, z0, z1, MX, MY).cut(box(MX - 20, MX + 20, MY - 20, MY, z0 - 1, z1 + 1)))
    for gz in GUIDE_Z: s = s.union(box(-EAR_W, 14, E0, E0 + EAR_H, gz - EAR_W, gz + EAR_W))
    s = s.cut(cylz(12.75, Z_ENC - 1, M_FACE, MX, MY))                                   # 모터 자리 (0.25 여유)
    s = s.cut(cylx(BRG_R, 12.9, 15.5, A0, 0)).cut(cylx(BRG_R, 28, 30.6, A0, 0)).cut(cylx(2.5, 12, 31, A0, 0))   # MF74ZZ ×2 (B, C)
    s = s.cut(cylz(4.5, M_FACE - 1, -6, MX, MY))                                        # 돌출부 Ø7 + 축
    for sx in (-1, 1):                                                                  # M3 긴 구멍 + 머리 자리(깊이 1.7)
        x0, x1 = sorted((MX + sx * M_SCREW[0] / 2, MX + sx * M_SCREW[1] / 2))
        for r, za in ((1.65, M_FACE - 1), (3.1, -6.5 - 1.7)):
            s = s.cut(cylz(r, za, -6, x0, MY)).cut(cylz(r, za, -6, x1, MY)).cut(box(x0, x1, MY - r, MY + r, za, -6))
    for gz in GUIDE_Z: s = s.cut(cyly(BUSH_R, E0 - 1, E0 + EAR_H + 1, 0, gz))           # LM4UU Ø8
    return s

# ---------------- 출력 웜기어 (모듈 1.0, 시제품용) ----------------
def _inv(a): return math.tan(a) - a
def _gear_profile(z, m, thin, alpha=math.radians(20)):
    rp = m * z / 2; rb = rp * math.cos(alpha); ra = rp + m; rf = rp - 1.25 * m
    half = math.pi / (2 * z) + _inv(alpha) - thin / rp; pts = []
    for i in range(z):
        th0 = 2 * math.pi * i / z; right = []
        for r in [rf] + ([rb] if rb > rf else []) + [rb + (ra - rb) * k / 10 for k in range(1, 11)]:
            a = math.acos(min(1, rb / r)) if r > rb else 0.0
            right.append((r, half - _inv(a)))
        for r, ang in right: pts.append((r * math.cos(th0 - ang), r * math.sin(th0 - ang)))
        for r, ang in reversed(right): pts.append((r * math.cos(th0 + ang), r * math.sin(th0 + ang)))
    return pts
def worm_local():
    """웜: 축 = z, 중심 원점. 한 줄 오른나사, 축 방향 모듈 1.0. 구멍은 FIT0522 D축(Ø4, 평면 3.5)용 D자."""
    p = math.pi * GEAR_M; rr = WORM_D1 / 2 - 1.25 * GEAR_M; t = math.tan(math.radians(20))
    w = lambda r: p / 4 - (r - WORM_D1 / 2) * t - GEAR_BL / 2
    z0 = -WORM_L / 2 - 2 * p; r0 = rr - 0.4
    pts = [(r0, z0 - w(r0)), (WORM_RA, z0 - w(WORM_RA)), (WORM_RA, z0 + w(WORM_RA)), (r0, z0 + w(r0))]
    helix = cq.Wire.makeHelix(pitch=p, height=WORM_L + 4 * p, radius=r0, center=cq.Vector(0, 0, z0))
    th = cq.Workplane("XZ").polyline(pts).close().sweep(cq.Workplane("XY").add(helix), isFrenet=True)
    s = th.intersect(cq.Workplane("XY").box(40, 40, WORM_L)).union(cylz(rr, -WORM_L / 2, WORM_L / 2))
    bore = cylz(2.1, -WORM_L / 2 - 1, WORM_L / 2 + 1).cut(box(1.6, 3, -3, 3, -WORM_L / 2 - 1, WORM_L / 2 + 1))   # D자 구멍 (평면까지 1.6)
    s = s.cut(bore)
    return s.cut(cq.Workplane("YZ").circle(1.25).extrude(8).translate((1.2, 0, -3.0)))      # M3 무두 멈춤나사 파일럿 Ø2.5 → D 평면
def wormwheel_local():
    """웜휠: 축 = z, 중심 원점. 16T 헬리컬(비틀림각 = 웜 리드각 4.76°)로 근사. Ø4 축 구멍 + M3 멈춤나사 파일럿."""
    rp = GEAR_M * WHEEL_Z / 2; beta = math.atan(GEAR_M / WORM_D1)
    tw = math.degrees(WHEEL_B * math.tan(beta) / rp)
    g = cq.Workplane("XY").polyline(_gear_profile(WHEEL_Z, GEAR_M, GEAR_BL / 2)).close().twistExtrude(WHEEL_B, tw).translate((0, 0, -WHEEL_B / 2))
    g = g.cut(cylz(2.05, -WHEEL_B, WHEEL_B))
    gap = math.radians(180.0 / WHEEL_Z)                                                   # 이 사이 골 방향으로 멈춤나사
    return g.cut(cq.Workplane("XY").add(cq.Solid.makeCylinder(1.25, 10, cq.Vector(0, 0, 0), cq.Vector(math.cos(gap), math.sin(gap), 0))))
def worm(): return worm_local().translate((MX, MY, 0))
def wormwheel(): return wormwheel_local().rotate((0, 0, 0), (0, 1, 0), 90).rotate((0, 0, 0), (1, 0, 0), WHEEL_PHASE).translate((MX, A0, 0))

def hub():   # v1과 같은 형상 (M2×4 PCD 12, M3 멈춤나사)
    h = cylx(9, -14.5, -11.5, A0, 0).union(cylx(5, -19.5, -14.5, A0, 0)).cut(cylx(2.05, -20, -11, A0, 0))
    for i in range(4):
        a = math.radians(45 + 90 * i); h = h.cut(cylx(1.1, -15, -11, A0 + 6 * math.cos(a), 6 * math.sin(a)))
    return h.cut(cyly(1.3, A0, A0 + 6, -17, 0))
def spacer(): return cyly(3.25, SEAT_Y, SEAT_Y + SPACER, 0, GUIDE_Z[0]).cut(cyly(2.15, SEAT_Y - 1, SEAT_Y + SPACER + 1, 0, GUIDE_Z[0]))

# ---------------- 구매품 (움직이는 것) ----------------
def fit0522():
    m = cylz(12.5, Z_GB, M_FACE, MX, MY).union(cylz(12.2, Z_CAN, Z_GB, MX, MY)).union(cylz(7, Z_ENC, Z_CAN, MX, MY))
    m = m.union(cylz(3.5, M_FACE, M_FACE + 2.5, MX, MY)).union(cylz(2, M_FACE + 2.5, M_FACE + 14.5, MX, MY).cut(box(MX + 1.5, MX + 3, MY - 3, MY + 3, M_FACE + 4.5, M_FACE + 15)))
    return m
def bought_moving():
    b = None
    for x0, x1, f0, f1 in ((13, 15.5, 12.4, 13), (28, 30.5, 30.5, 31.1)):
        r = cylx(3.5, x0, x1, A0, 0).union(cylx(4.1, f0, f1, A0, 0)).cut(cylx(2, x0 - 1, x1 + 1, A0, 0)); b = r if b is None else b.union(r)
    bush = None
    for gz in GUIDE_Z:
        r = cyly(4.0, E0, E0 + EAR_H, 0, gz).cut(cyly(2, E0 - 1, E0 + EAR_H + 1, 0, gz)); bush = r if bush is None else bush.union(r)
    # 웜·웜휠은 간섭 검사에서 이끝 원통(외곽)으로 본다. 실제 이 형상은 worm(), wormwheel()
    return dict(wheel=cylx(WR, -WW / 2, WW / 2, A0, 0), wormwheel=cylx(WHEEL_RA, MX - WHEEL_B / 2, MX + WHEEL_B / 2, A0, 0),
                worm=cylz(WORM_RA, -WORM_L / 2, WORM_L / 2, MX, MY),
                axle=cylx(2, -19.5, AXLE_X1, A0, 0), bearing=b, bushing=bush, motor=fit0522(), plug=cylz(7, Z_ENC - PLUG, Z_ENC, MX, MY))
def cutter_prims():
    return slider_prims() + [("cylx", WR, -WW / 2, WW / 2, A0, 0), ("cylx", 9, -14.5, -11.5, A0, 0), ("cylx", 5, -19.5, -14.5, A0, 0),
            ("cylx", WHEEL_RA, MX - WHEEL_B / 2, MX + WHEEL_B / 2, A0, 0), ("cylz", WORM_RA, -WORM_L / 2, WORM_L / 2, MX, MY), ("cylx", 2, -19.5, AXLE_X1, A0, 0),
            ("cylx", 4.1, 12.4, 13, A0, 0), ("cylx", 4.1, 30.5, 31.1, A0, 0),
            ("cylz", 12.5, Z_GB, M_FACE, MX, MY), ("cylz", 12.2, Z_CAN, Z_GB, MX, MY), ("cylz", 7, Z_ENC - PLUG, Z_CAN, MX, MY)]

# ---------------- 고정부 (출력) ----------------
def rods_unit():
    r = None
    for gz in GUIDE_Z:
        q = cyly(2, ROD_Y0, ROD_TOP, 0, gz).union(cyly(3.5, ROD_TOP - 2, ROD_TOP, 0, gz)); r = q if r is None else r.union(q)
    return r
def base():
    """뒷판 + 가운데 기둥 + 스프링 받침 블록 (한 덩어리 출력)."""
    s = cylz(PLATE_R, Z_REAR, Z_REAR + 3).union(cylz(7.5, Z_REAR + 3, Z_FRONT - FC_T))
    blk = cylz(13, -45, -20)
    for k in range(3): blk = blk.cut(rotz(box(-30, 30, SEAT_Y, 40, -46, -19), 120 * k))
    s = s.union(blk)
    for k in range(3):
        for gz in GUIDE_Z: s = s.cut(rotz(cyly(1.95, ROD_Y0, SEAT_Y + 0.5, 0, gz), 120 * k))   # 봉 압입, 깊이 4.5
        s = s.cut(rotz(box(MX - 5, MX + 5, 9, 45, Z_REAR - 1, Z_REAR + 4), 120 * k))          # 모터선 홈 (행정 범위)
        x, y = 22 * math.sin(math.radians(JOINT_ANG)), 22 * math.cos(math.radians(JOINT_ANG))
        s = s.cut(rotz(cylz(1.65, Z_REAR - 1, Z_REAR + 4, x, y), 120 * k))                   # 관절 연결 M3 (관절 미정)
        for ang in SCREW_ANG: s = s.cut(rotz(radial(0.8, PLATE_R - 6, PLATE_R + 1, ang, Z_REAR + 1.5), 120 * k))   # 덮개 M2 파일럿
    s = s.cut(cylz(4, Z_REAR - 1, Z_FRONT))                                                   # 배선 구멍 Ø8
    for ang in (JOINT_ANG, JOINT_ANG + 180):                                               # 앞판 고정 M2 파일럿 (기둥 벽)
        a = math.radians(ang); s = s.cut(cylz(0.8, Z_FRONT - FC_T - 8, Z_FRONT - FC_T + 1, 5.75 * math.sin(a), 5.75 * math.cos(a)))
    return s
def front_cap():
    s = cylz(PLATE_R, Z_FRONT - FC_T, Z_FRONT).cut(cylz(4, Z_FRONT - FC_T - 1, Z_FRONT + 1))
    for ang in (JOINT_ANG, JOINT_ANG + 180):
        a = math.radians(ang); s = s.cut(cylz(1.1, Z_FRONT - FC_T - 1, Z_FRONT + 1, 5.75 * math.sin(a), 5.75 * math.cos(a)))
    for k in range(3):
        for ang in SCREW_ANG: s = s.cut(rotz(radial(0.8, PLATE_R - 6, PLATE_R + 1, ang, Z_FRONT - 1.5), 120 * k))
    return s
D_INSERT = 40.0                                               # 유닛을 덮개 바깥에서 반지름 방향으로 끼워 넣는 경로까지 창에 포함
def cover_cutter(c=1.0):
    u = None
    for p in cutter_prims():
        q = swept(p, D_MIN, D_INSERT, c); u = q if u is None else u.union(q)
    for gz in GUIDE_Z:
        u = u.union(cyly(2 + 0.6, ROD_Y0, ROD_TOP + 1, 0, gz)).union(cyly(3.5 + 0.6, ROD_TOP - 2.6, ROD_TOP + 1, 0, gz))
    return u
COVER_INFO = {}
def tri_prism(d, rv, z0, z1, psi=None):
    """둥근 삼각 기둥: 세 평면(중심에서 d, 법선 psi + 120k) ∩ 원(rv)."""
    psi = TRI_PSI if psi is None else psi
    pts = [(2 * d * math.cos(math.radians(psi + 60 + 120 * k)), 2 * d * math.sin(math.radians(psi + 60 + 120 * k))) for k in range(3)]
    return cylz(rv, z0, z1).intersect(cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0).translate((0, 0, z0)))
def window():
    """유닛 좌표의 창 (반지름 방향으로 뚫음). 구간마다 부품이 실제로 지나는 폭만큼만 뚫는다 (여유 1).
    뒤부터: 모터 폭 → 귀 + 모터 폭 → 허브 + 모터 폭 → 허브 + 웜기어 폭 → 바퀴 폭."""
    wx = WW / 2 + 1.0; wz = WR + 1.0
    w = box(MX - 13.5, WIN_X1, 0, 70, WIN_ZR, WIN_ZE + 2).union(box(WIN_XE, WIN_X1, 0, 70, WIN_ZE - 0.9, WIN_ZW + 2)).union(
        box(WIN_X0, WIN_X1, 0, 70, WIN_ZW, WIN_ZM)).union(box(WIN_X0, WIN_XG, 0, 70, WIN_ZM - 2, WIN_ZG)).union(box(-wx, wx, 0, 70, WIN_ZG - 2.5, wz))
    return w.edges("|Y").fillet(WIN_R).union(box(-wx, wx, 0, 70, wz - WIN_R - 1, wz))      # 바퀴 창 앞 모서리는 각지게 (바퀴 여유 확보)
def cover():
    """삼각 외피 (한 덩어리). 바깥 = 세 평면(TRI_D) ∩ 원(TRI_RV), 벽 두께 TRI_T. 양 끝 고리가 앞·뒷판(Ø74)을 감싼다.
    꼭짓점(둥근 모서리)은 유닛 사이 빈 자리에, 평면은 바퀴 유닛 쪽에 둔다. 유닛은 평면의 창으로 드나든다.
    창은 유닛마다 하나(window). 유닛 사이에는 꼭짓점을 따라 레일 세 줄이 남고, 레일은 안쪽을 채워 두껍게 한다.
    창이 실제 부품 경로(cover_cutter: 행정 + 끼우는 경로, 여유 1)를 모두 포함하는지 COVER_INFO 에 남긴다."""
    z0, z1 = Z_REAR + 3, Z_FRONT - FC_T
    cov = tri_prism(TRI_D, TRI_RV, Z_REAR, Z_FRONT).cut(cylz(COV_RI, Z_REAR - 1, Z_FRONT + 1))  # 양 끝 고리
    cov = cov.cut(tri_prism(TRI_D - TRI_T, TRI_RV - TRI_T, z0, z1))                              # 가운데는 벽 두께만 남김
    phi_a = math.degrees(math.acos(WIN_X1 / (TRI_RV - TRI_EDGE)))                                # 레일의 모터 쪽 가장자리 각 (두께 TRI_EDGE 인 곳)
    fill = tri_prism(TRI_D, TRI_RV, z0, z1).cut(cylz(COV_RI + 0.5, z0 - 1, z1 + 1))
    rail = fill.intersect(box(WIN_X1, 60, -40, 40, z0, z1)).intersect(rotz(box(-60, WIN_X0, 0, 70, z0, z1), 240)).intersect(rotz(box(30, 60, -40, 0, z0, z1), phi_a))
    for k in range(3): cov = cov.union(rotz(rail, 120 * k))                                      # 레일 안쪽 채움 (유닛 0 창과 옆 유닛 창 사이)
    w = window(); trim = rotz(box(30, 60, 0, 22, WIN_ZR, WIN_ZM), phi_a)                         # trim: 창 가장자리의 얇은 날을 잘라 냄
    for k in range(3): cov = cov.cut(rotz(w, 120 * k)).cut(rotz(trim, 120 * k))
    sol = cov.solids().vals()
    COVER_INFO["dropped_mm3"] = round(sum(x.Volume() for x in sol) - max(x.Volume() for x in sol), 1)
    cov = cq.Workplane("XY").add(max(sol, key=lambda x: x.Volume()))
    cut = cover_cutter()
    COVER_INFO["path_overlap_mm3"] = round(sum(x.Volume() for k in range(3) for x in cov.intersect(rotz(cut, 120 * k)).solids().vals()), 2)
    for k in range(3):                                            # 앞·뒷판 M2
        for ang in SCREW_ANG:
            for z in (Z_REAR + 1.5, Z_FRONT - 1.5): cov = cov.cut(rotz(radial(1.1, COV_RI - 1, TRI_RV + 1, ang, z), 120 * k))
    return cov
def cover_round():
    """[이전 덮개, 대안으로 남김] 한 덩어리 원통 덮개. 바깥 Ø78. 앞뒤 판을 감싸는 양 끝은 두께 2, 그 사이는 안쪽 반지름 35까지 채운 뒤
    움직이는 부품이 행정 전체에서 지나는 공간(+여유 1)과 가이드봉 자리를 잘라 창을 낸다."""
    cov = tube(COV_RO, COV_RI, Z_REAR, Z_FRONT).union(tube(COV_RO, COV_RI_IN, Z_REAR + 3, Z_FRONT - FC_T))
    cut = cover_cutter()
    for k in range(3): cov = cov.cut(rotz(cut, 120 * k))
    sol = cov.solids().vals()                                     # 창 사이에 떨어진 작은 조각은 버림 (본체만)
    cov = cq.Workplane("XY").add(max(sol, key=lambda s: s.Volume()))
    for k in range(3):
        for ang in SCREW_ANG:
            for z in (Z_REAR + 1.5, Z_FRONT - 1.5): cov = cov.cut(rotz(radial(1.1, COV_RI - 1, COV_RO + 1, ang, z), 120 * k))
    return cov
def cover_parts(): return [cover()]        # 그림 스크립트용

PRINT = dict(slider=slider, hub=hub, spacer=spacer, base=base, front_cap=front_cap, cover=cover, worm=worm_local, wormwheel=wormwheel_local)
QTY = dict(slider=3, hub=3, spacer=6, base=1, front_cap=1, cover=1, worm=3, wormwheel=3)
def gear_mesh_overlap():
    """실제 이 형상끼리 설치 위상에서 겹치는 부피(mm3)와, 웜휠을 반 피치 돌렸을 때의 겹침(비교용)."""
    w = worm(); g = wormwheel()
    half = wormwheel_local().rotate((0, 0, 0), (0, 1, 0), 90).rotate((0, 0, 0), (1, 0, 0), WHEEL_PHASE + 180.0 / WHEEL_Z).translate((MX, A0, 0))
    f = lambda a, b: round(sum(s.Volume() for s in a.intersect(b).solids().vals()), 3)
    return f(w, g), f(w, half)

# ---------------- 간섭 검사 ----------------
def vol(a, b):
    A, B = a.val().BoundingBox(), b.val().BoundingBox()
    if A.xmax <= B.xmin or B.xmax <= A.xmin or A.ymax <= B.ymin or B.ymax <= A.ymin or A.zmax <= B.zmin or B.zmax <= A.zmin: return 0.0
    return sum(s.Volume() for s in a.intersect(b).solids().vals())
def unit_parts(P, mv):
    return dict(slider=P["slider"], hub=P["hub"], **mv)
def place(parts, k, d): return {n: rotz(p.translate((0, d, 0)), 120 * k) for n, p in parts.items()}
OK_PAIRS = [{"axle", "wheel"}, {"axle", "hub"}, {"axle", "wormwheel"}, {"axle", "bearing"}, {"worm", "motor"}, {"worm", "wormwheel"},
            {"bushing", "slider"}, {"motor", "plug"}]
def check(P, log):
    mv = bought_moving(); U = unit_parts(P, mv); bad = []
    def rec(tag, a, b, v):
        if v > 0.01: bad.append(dict(check=tag, a=a, b=b, mm3=round(v, 3)))
    for a, b in itertools.combinations(U, 2):                       # 유닛 내부
        if {a, b} not in OK_PAIRS: rec("unit", a, b, vol(U[a], U[b]))
    log(f"유닛 내부 검사 완료, 문제 {len(bad)}")
    fixed = dict(base=P["base"], front_cap=P["front_cap"], cover=P["cover"])
    for k in range(3): fixed[f"rods{k}"] = rotz(rods_unit(), 120 * k)
    sp = None
    for k in range(3):
        for gz in GUIDE_Z:
            q = rotz(P["spacer"].translate((0, 0, gz - GUIDE_Z[0])), 120 * k); sp = q if sp is None else sp.union(q)
    for d in D_CHECK:                                                 # 유닛 0 ↔ 고정부
        u0 = place(U, 0, d); F = dict(fixed)
        if d >= D_SOLID - 1e-6: F["spacers"] = sp                     # 내경 120 설정에서만 스페이서
        for n, p in u0.items():
            for fn, fp in F.items():
                if (n == "bushing" and fn == "rods0"): continue
                rec(f"d={d:.2f} u0↔fixed", n, fn, vol(p, fp))
        log(f"d={d:.2f} 고정부 검사 완료, 누적 문제 {len(bad)}")
    for d0, d1 in itertools.product(D_CHECK, D_CHECK):               # 유닛 0 ↔ 유닛 1 (대칭으로 나머지 쌍 대표)
        u0, u1 = place(U, 0, d0), place(U, 1, d1)
        for n0, p0 in u0.items():
            for n1, p1 in u1.items(): rec(f"d0={d0:.2f} d1={d1:.2f} u0↔u1", n0, n1, vol(p0, p1))
    log(f"유닛 사이 검사 완료, 누적 문제 {len(bad)}")
    return bad

if __name__ == "__main__":
    do_check = "--no-check" not in sys.argv
    lines = []
    def log(s): print(s, flush=True); lines.append(s)
    log(f"CadQuery {cq.__version__}, Python {sys.version.split()[0]}")
    log(f"모듈 길이 {Z_FRONT - Z_REAR:.1f}, 행정 d {D_MIN}~{D_MAX}, 스프링 한계 d {D_SOLID:.2f}, 이음부 d {D_JOINT:.2f}")
    P = {n: f() for n, f in PRINT.items()}
    log(f"덮개(삼각 외피): 버린 떨어진 조각 {COVER_INFO['dropped_mm3']} mm3, 창과 부품 경로(행정 + 끼우는 경로, 여유 1)의 겹침 {COVER_INFO['path_overlap_mm3']} mm3 (0 이어야 함)")
    (OUT / "stl").mkdir(exist_ok=True); (OUT / "step").mkdir(exist_ok=True)
    for old in ("cover_petal.stl", "cover_petal.step", "cover_tri.stl", "cover_tri.step"):   # 이전 이름의 덮개 파일 정리
        f = OUT / ("stl" if old.endswith("stl") else "step") / old
        if f.exists(): f.unlink()
    for n, p in P.items():
        sol = p.solids().size(); ok = p.val().isValid(); bb = p.val().BoundingBox()
        log(f"{n} ×{QTY[n]}: 솔리드 {sol}, 유효 {ok}, {bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f} mm, {sum(s.Volume() for s in p.solids().vals())/1000:.1f} cm3")
        cq.exporters.export(p, str(OUT / "stl" / f"{n}.stl"), tolerance=0.02, angularTolerance=0.15)
        cq.exporters.export(p, str(OUT / "step" / f"{n}.step"))
    cr = cover_round()                                                 # 이전 원통 덮개: 파일만 함께 냄 (조립·검사에는 넣지 않음)
    log(f"cover_round (대안, 검사 제외): 솔리드 {cr.solids().size()}, 유효 {cr.val().isValid()}, {sum(s.Volume() for s in cr.solids().vals())/1000:.1f} cm3")
    cq.exporters.export(cr, str(OUT / "stl" / "cover_round.stl"), tolerance=0.02, angularTolerance=0.15)
    cq.exporters.export(cr, str(OUT / "step" / "cover_round.step"))
    asm = cq.Assembly(name="front_module_v2")
    col = dict(slider="lightskyblue", hub="khaki", wheel="royalblue", wormwheel="goldenrod", worm="goldenrod", axle="gray40", bearing="gray50",
               bushing="gray60", motor="gray70", plug="green", base="slategray", front_cap="gray80", petal="gray85", rods="gray30", spacer="plum")
    asm.add(P["base"], name="base", color=cq.Color(col["base"])); asm.add(P["front_cap"], name="front_cap", color=cq.Color(col["front_cap"]))
    asm.add(P["cover"], name="cover", color=cq.Color(col["petal"]))
    U = unit_parts(P, bought_moving())
    # 조립 STEP에는 웜·웜휠을 이끝 원통으로 넣는다 (실제 이 형상 3세트를 넣으면 파일이 11 MB가 됨). 이 형상은 worm.step, wormwheel.step
    m0, m1 = gear_mesh_overlap()
    log(f"웜기어 맞물림: 설치 위상 겹침 {m0} mm3, 반 피치 어긋난 위상 겹침 {m1} mm3 (감속 1:{WHEEL_Z}, 축간 {CD:.1f})")
    for k in range(3):
        asm.add(rotz(rods_unit(), 120 * k), name=f"rods{k}", color=cq.Color(col["rods"]))
        for i, gz in enumerate(GUIDE_Z): asm.add(rotz(P["spacer"].translate((0, 0, gz - GUIDE_Z[0])), 120 * k), name=f"spacer{k}_{i}", color=cq.Color(col["spacer"]))
        for n, p in place(U, k, 0).items(): asm.add(p, name=f"u{k}_{n}", color=cq.Color(col[n]))
    asm.save(str(OUT / "step" / "front_module_v2_assembly.step"))
    log("STL·STEP 저장 완료")
    if do_check:
        bad = check(P, log)
        log("간섭: " + ("없음" if not bad else json.dumps(bad, ensure_ascii=False)))
        (OUT / "check_results.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
