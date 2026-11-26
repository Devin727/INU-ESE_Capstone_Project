"""앞 모듈 v2 배치도 + 서스펜션 유닛 (내경 120 시연 기준, C202 + 스페이서 9). 단위 mm.
실행: python 앞모듈_v2_배치도_draw.py <배치도.png> <배치도.pdf> <유닛.png>     (cadquery 필요)
치수와 덮개 모양은 ../cad/v2/front_module_v2.py 에서 읽는다 (CAD와 같은 값). 간섭 검사는 그 스크립트에서 한다.
"""
import sys, math, pathlib
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle, Wedge
from matplotlib import font_manager as fm

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "cad" / "v2"))
import front_module_v2 as F
import cadquery as cq

for f in ("C:/Windows/Fonts/malgun.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
    try: fm.fontManager.addfont(f); plt.rcParams["font.family"] = fm.FontProperties(fname=f).get_name(); break
    except Exception: pass
plt.rcParams["axes.unicode_minus"] = False

# ---------------- 스프링 설정 (C202 + 스페이서) ----------------
K, L_FREE, L_MIN = 1.41, 16.0, 9.09       # C202: 약 1.41 N/mm [계산], 자유 16, 허용 압축 높이 9.09 [판매]
SEAT = F.SEAT_Y + F.SPACER                # 스프링 아랫면 18
def spring_len(a): return a - F.EAR_OFF - SEAT
def wheel_force(a): return 2 * K * max(L_FREE - spring_len(a), 0)    # 바퀴당 (스프링 2개)
A_SET = F.A0                              # 내경 120 접촉 40.95
A_JOINT = F.A0 + F.D_JOINT                # 이음부 Ø123.8 접촉 42.85
A_SOLID = F.A0 + F.D_SOLID                # 스프링 한계 38.04
L_MOD = F.Z_FRONT - F.Z_REAR

WR, WW, MX, CD = F.WR, F.WW, F.MX, F.CD
C = dict(pipe="#9aa7b4", wheel="#3f6fb5", motor="#b9c0c9", spring="#e07b1a", rod="#555", ear="#2f9e6e", gear="#c9a227",
         plate="#7fb3e6", hub="#e8e0c8", fix="#8c96a3", cover="#5b6573", spacer="#c8a2c8", slider="#7fb3e6")
def Rm(deg): t = np.radians(deg); return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
def rc(x0, x1, y0, y1): return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
def dim(ax, p, q, txt, off=(0, 0), ha="center", va="bottom", fs=8.5):
    ax.annotate("", xy=p, xytext=q, arrowprops=dict(arrowstyle="<->", lw=.8, color="k"))
    ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], txt, ha=ha, va=va, fontsize=fs, bbox=dict(fc="w", ec="none", pad=.6, alpha=.85))
def lab(ax, xy, xt, txt, fs=8.5, ha="left"):
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, ha=ha, va="center", arrowprops=dict(arrowstyle="-", lw=.7, color="#333"))
def zig(ax, x, y0, y1, w=3.25, n=12):
    yy = np.linspace(y0, y1, n); ax.plot(x + w * np.array([(-1) ** i for i in range(n)]), yy, color=C["spring"], lw=1.8, zorder=8)

# ---------------- 덮개: CAD 조각에서 직접 샘플 ----------------
PETALS = [p.val() for p in F.cover_parts()]
RR = np.arange(36.25, F.TRI_RV + .3, .25)                        # 덮개(삼각 외피)를 찾는 반지름 범위
def cover_at(x, y, z): return any(p.isInside(cq.Vector(x, y, z)) for p in PETALS)
def cover_span(phi_deg, z):
    """각 phi 방향에서 덮개가 있는 반지름 구간 (없으면 None)."""
    c, s = np.cos(np.radians(phi_deg)), np.sin(np.radians(phi_deg)); hit = [r for r in RR if cover_at(r * c, r * s, z)]
    return (min(hit) - .125, max(hit) + .125) if hit else None
def runs(flags, vals):
    out, st = [], None
    for v, f in zip(vals, flags):
        if f and st is None: st = v
        if not f and st is not None: out.append((st, v)); st = None
    if st is not None: out.append((st, vals[-1]))
    return out
def cover_front(ax, zc, ls, fc):
    phi = np.arange(0, 360, 1.0); sp = [cover_span(p, zc) for p in phi]; i = 0
    while i < len(phi):
        if sp[i] is None: i += 1; continue
        j = i
        while j + 1 < len(phi) and sp[j + 1] is not None: j += 1
        a = np.radians(phi[i:j + 1]); ro = np.array([sp[k][1] for k in range(i, j + 1)]); ri = np.array([sp[k][0] for k in range(i, j + 1)])
        pts = np.r_[np.c_[ro * np.cos(a), ro * np.sin(a)], np.c_[ri * np.cos(a), ri * np.sin(a)][::-1]]
        ax.add_patch(Polygon(pts, closed=True, fc=fc, ec="k", lw=.8, ls=ls, zorder=7, alpha=.9)); i = j + 1
ZS = np.arange(F.Z_REAR, F.Z_FRONT + .01, .5)
_side = [cover_span(90, z) for z in ZS]                            # 유닛 0 중심선(바퀴 방향) 위치
COVER_SIDE = [(z0, z1, min(s[0] for s, z in zip(_side, ZS) if s and z0 <= z <= z1), max(s[1] for s, z in zip(_side, ZS) if s and z0 <= z <= z1))
              for z0, z1 in runs([s is not None for s in _side], ZS)]

# ---------------- 유닛 그리기 (CAD 치수) ----------------
def unit_front(ax, a, M=np.eye(2), full=True):
    P = lambda q: q @ M.T; my = a - CD; e = a - F.EAR_OFF
    ax.add_patch(Circle(M @ np.array([MX, my]), 12.5, fc=C["motor"], ec="k", lw=.9, ls="--", alpha=.7, zorder=2))
    ax.add_patch(Polygon(P(rc(17, 33, my + 13, my + 16)), fc=C["slider"], ec="k", lw=.5, ls="--", alpha=.6, zorder=3))   # 모터 위 막대 (뒤)
    ax.add_patch(Polygon(P(rc(-2, 2, F.ROD_Y0, F.ROD_TOP)), fc=C["rod"], ec="k", lw=.4, zorder=3))
    if full:
        ax.add_patch(Polygon(P(rc(-3.25, 3.25, F.SEAT_Y, SEAT)), fc=C["spacer"], ec="k", lw=.5, zorder=4))
        ax.add_patch(Polygon(P(rc(-3.25, 3.25, SEAT, e)), fc="none", ec=C["spring"], lw=1.4, hatch="----", zorder=4))
    ax.add_patch(Polygon(P(rc(-F.EAR_W, 14, e, e + F.EAR_H)), fc=C["ear"], ec="k", lw=.5, alpha=.8, zorder=4))         # 귀 + 팔 (뒤)
    ax.add_patch(Polygon(P(rc(-WW / 2, WW / 2, a - WR, a + WR)), fc=C["wheel"], ec="k", lw=.9, alpha=.85, zorder=5))
    ax.add_patch(Polygon(P(rc(-19.5, -11.5, a - 9, a + 9)), fc=C["hub"], ec="k", lw=.6, zorder=5))
    ax.add_patch(Polygon(P(rc(13, 21.5, a - 5, a + 5.05)), fc=C["plate"], ec="k", lw=.6, zorder=5))               # 지지판 B (베어링 1)
    ax.add_patch(Polygon(P(rc(28, 30.5, a - 5, a + 5.05)), fc=C["plate"], ec="k", lw=.6, zorder=5))               # 지지판 C (베어링 1)
    ax.add_patch(Polygon(P(rc(MX - F.WHEEL_B / 2, MX + F.WHEEL_B / 2, a - F.WHEEL_RA, a + F.WHEEL_RA)), fc=C["gear"], ec="k", lw=.6, zorder=5))
    ax.add_patch(Circle(M @ np.array([MX, my]), F.WORM_RA, fc=C["gear"], ec="k", lw=.6, alpha=.85, hatch="///", zorder=4))
    ax.plot(*P(np.array([[-19.5, a], [F.AXLE_X1, a]])).T, color="k", lw=.8, ls="-.", zorder=6)

def unit_side(ax, a, cover=True):
    """유닛 0 옆모습 (x 쪽에서 본 투영, 모터는 옆으로 25 떨어져 있음)."""
    my = a - CD; e = a - F.EAR_OFF
    if cover:
        for z0, z1, r0, r1 in COVER_SIDE: ax.add_patch(Rectangle((z0, r0), z1 - z0, r1 - r0, fc=C["cover"], ec="k", lw=.6, zorder=2))
    for z0, z1, y0, y1 in ((-9.5, 6, a - 5, a + 5.05), (-9.5, -6.5, my - 5, a - 5), (-44, -9.3, my + 13, my + 16), (-9.4, 5.05, my + 13, a + 5.05)) + \
                          tuple((gz - F.EAR_W, gz + F.EAR_W, e, e + F.EAR_H) for gz in F.GUIDE_Z):
        ax.add_patch(Rectangle((z0, y0), z1 - z0, y1 - y0, fc=C["slider"], ec="k", lw=.6, alpha=.45, zorder=3))
    for z0, z1 in ((-29, -21), (-44, -36)):                                                    # 모터 받침 (위 반원)
        ax.add_patch(Rectangle((z0, my), z1 - z0, 14.75, fc=C["slider"], ec="k", lw=.6, alpha=.45, zorder=3))
    ax.add_patch(Rectangle((F.Z_GB, my - 12.5), F.M_FACE - F.Z_GB, 25, fc=C["motor"], ec="k", lw=1, zorder=4))
    ax.add_patch(Rectangle((F.Z_CAN, my - 12.2), F.Z_GB - F.Z_CAN, 24.4, fc=C["motor"], ec="k", lw=1, zorder=4))
    ax.add_patch(Rectangle((F.Z_ENC, my - 7), F.Z_CAN - F.Z_ENC, 14, fc="#5aa469", ec="k", lw=.8, zorder=4))
    ax.add_patch(Rectangle((F.Z_ENC - F.PLUG, my - 7), F.PLUG, 14, fc="none", ec="k", lw=.8, ls="--", hatch="//", zorder=4))
    ax.add_patch(Rectangle((-5, my - F.WORM_RA), 10, 2 * F.WORM_RA, fc=C["gear"], ec="k", lw=.7, hatch="///", zorder=6))
    ax.add_patch(Circle((0, a), WR, fc=C["wheel"], ec="k", lw=1, alpha=.8, zorder=5))
    ax.add_patch(Circle((0, a), 8.0, fc="none", ec=C["gear"], lw=1.8, ls="--", zorder=6)); ax.add_patch(Circle((0, a), 2, fc="k", zorder=7))
    for gz in F.GUIDE_Z:
        ax.add_patch(Rectangle((gz - 5, 7.5), 10, F.SEAT_Y - 7.5, fc=C["fix"], ec="k", lw=.6, zorder=6))
        ax.add_patch(Rectangle((gz - 2, F.ROD_Y0), 4, F.ROD_TOP - F.ROD_Y0, fc=C["rod"], ec="k", lw=.5, zorder=7))
        ax.add_patch(Rectangle((gz - 3.25, F.SEAT_Y), 6.5, F.SPACER, fc=C["spacer"], ec="k", lw=.5, zorder=8))
        zig(ax, gz, SEAT, e)
        ax.add_patch(Rectangle((gz - 3.5, F.ROD_TOP - 2), 7, 2, fc="#222", zorder=9))

def body_side(ax, wall=60, x0=None, x1=None):
    x0 = F.Z_REAR - 25 if x0 is None else x0; x1 = F.Z_FRONT + 15 if x1 is None else x1
    ax.add_patch(Rectangle((x0, wall), x1 - x0, 5, fc="#eef1f4", ec=C["pipe"])); ax.plot([x0, x1], [wall, wall], "k", lw=1.4)
    zs0, zs1 = F.Z_REAR + 3, F.Z_FRONT - F.FC_T
    ax.add_patch(Rectangle((zs0, -7.5), zs1 - zs0, 15, fc="#6d7785", ec="k", lw=.8)); ax.add_patch(Rectangle((zs0, -4), zs1 - zs0, 8, fc="w", ec="k", lw=.5))
    for z0 in (F.Z_REAR, F.Z_FRONT - F.FC_T): ax.add_patch(Rectangle((z0, -F.PLATE_R), 3, 2 * F.PLATE_R, fc=C["fix"], ec="k", lw=.8))

# ======================= Figure 1: 배치도 =======================
def fig_layout(paths):
    fig = plt.figure(figsize=(19, 11.5), dpi=110)
    fig.suptitle("앞 모듈 배치도 v2 (내경 120 시연 기준)  —  단위 mm", fontsize=17, fontweight="bold", y=.975)
    ax = fig.add_axes([.01, .07, .44, .84]); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("① 정면 단면 (앞에서 본 모습, 내경 120에서 바퀴가 벽에 닿은 상태)", fontsize=13, loc="left")
    ax.add_patch(Circle((0, 0), 67, fc="#eef1f4", ec=C["pipe"])); ax.add_patch(Circle((0, 0), 60, fc="w", ec="k", lw=1.4))
    ax.add_patch(Circle((0, 0), 61.9, fc="none", ec="k", lw=.6, ls=":"))
    ax.add_patch(Circle((0, 0), F.PLATE_R, fc="none", ec=C["fix"], lw=1.2, ls="--"))
    for k in range(3): unit_front(ax, A_SET, Rm(120 * k))
    cover_front(ax, 0, "-", C["cover"]); cover_front(ax, -50, "--", "none")
    ax.add_patch(Circle((0, 0), 7.5, fc="#6d7785", ec="k", lw=1, zorder=9)); ax.add_patch(Circle((0, 0), 4, fc="w", ec="k", lw=.8, zorder=9))
    dim(ax, (-60, -72), (60, -72), "배관 내경 Ø120 (점선: 이음부 Ø123.8)", va="top", off=(0, -1))
    for x in (-60, 60): ax.plot([x, x], [0, -74], color="k", lw=.4, ls=":")
    dim(ax, (-F.TRI_RV, -64), (F.TRI_RV, -64), f"삼각 외피: 평면 거리 {F.TRI_D:g}, 최대 반지름 {F.TRI_RV:g} (한 덩어리, 유닛마다 창)")
    for x in (-F.TRI_RV, F.TRI_RV): ax.plot([x, x], [0, -66], color="k", lw=.4, ls=":")
    dim(ax, (-11.5, 64), (11.5, 64), "바퀴 폭 23")
    lab(ax, (8, 56), (26, 72), "옴니휠 Ø38.1")
    lab(ax, (MX, A_SET - CD - 9), (58, 18), f"모터 FIT0522 Ø25 (길이 방향, 바퀴 뒤)\n바퀴 중심선에서 옆으로 {MX:.0f}")
    lab(ax, (MX, A_SET + 6), (50, 56), f"웜휠 (출력, 모듈 {F.GEAR_M:.1f}, {F.WHEEL_Z}T)")
    lab(ax, (MX + 4, A_SET - CD), (62, 34), "웜 (모터 D축에 직접)")
    lab(ax, (-15.5, A_SET + 5), (-46, 66), "허브 (M2×4, PCD 12)", ha="right")
    lab(ax, (17, A_SET + 3), (-6, 76), "지지판 B·C (MF74ZZ 각 1, 베어링 간격 15)", ha="center")
    lab(ax, (0, 14), (-50, -10), f"가이드봉 Ø4 + 스페이서 {F.SPACER:.0f}\n+ 스프링 C202 (바퀴당 2개, 바퀴 뒤)", ha="right")
    lab(ax, ((F.TRI_RV - 2) * np.cos(np.radians(132)), (F.TRI_RV - 2) * np.sin(np.radians(132))), (-66, 24), "삼각 외피 (진한 회색: z 0 단면\n점선: 모터 구간 z -50)", ha="right")
    lab(ax, (-25, -24), (-66, -40), f"앞·뒷판 Ø{2*F.PLATE_R:.0f} (회색 점선)", ha="right")
    lab(ax, (3, -2), (-66, -54), "가운데 기둥 Ø15 / 안 Ø8 (배선)", ha="right")
    ax.text(0, -84, "점선 원 = 모터, 점선 사각 = 모터 위 막대 (바퀴 뒤, 투영). 전자 부품(MDD3A·ESP32)은 이 모듈에 없음", ha="center", fontsize=9, color="#444")
    ax.set_xlim(-118, 98); ax.set_ylim(-90, 82)
    ax = fig.add_axes([.46, .36, .53, .55]); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("② 측면 단면 (위쪽 바퀴 유닛 1개, 왼쪽이 뒤 · 오른쪽이 앞)", fontsize=13, loc="left")
    body_side(ax); ax.plot([F.Z_REAR - 25, F.Z_FRONT + 15], [-60, -60], "k", lw=1.4)
    unit_side(ax, A_SET)
    ax.add_patch(Rectangle((F.Z_FRONT, -8), 3, 16, fc="#f0c040", ec="k", lw=.8))
    ax.plot([F.Z_REAR - 25, F.Z_FRONT + 15], [0, 0], color="k", lw=.5, ls="-.")
    dim(ax, (F.Z_REAR, -48), (F.Z_FRONT, -48), f"모듈 길이 {L_MOD:.1f} (앞판~뒷판)", va="top", off=(0, -1))
    for z in (F.Z_REAR, F.Z_FRONT): ax.plot([z, z], [-35, -50], color="k", lw=.4, ls=":")
    dim(ax, (F.Z_ENC, -30), (F.M_FACE, -30), "모터·엔코더 63", va="top", off=(0, -1))
    dim(ax, (F.Z_ENC - F.PLUG, -30), (F.Z_ENC, -30), f"플러그 {F.PLUG:.0f}\n(가정)", va="top", off=(0, -1), fs=7.5)
    dim(ax, (F.M_FACE, -22), (0, -22), "9.5", va="top", off=(0, -1)); dim(ax, (0, -22), (F.Z_FRONT, -22), f"{F.Z_FRONT:.0f}", va="top", off=(0, -1))
    for z in (F.Z_ENC, F.M_FACE, 0): ax.plot([z, z], [A_SET - CD - 12.5, -32], color="k", lw=.4, ls=":")
    dim(ax, (F.GUIDE_Z[1], 54), (F.GUIDE_Z[0], 54), "15"); ax.text(sum(F.GUIDE_Z) / 2, 58, "가이드봉 간격", ha="center", fontsize=8)
    for z in F.GUIDE_Z: ax.plot([z, z], [F.ROD_TOP, 56], color="k", lw=.4, ls=":")
    dim(ax, (F.Z_REAR - 12, 0), (F.Z_REAR - 12, A_SET), "바퀴 축\n높이 40.95", ha="right", va="center", off=(-1.5, 6))
    dim(ax, (F.Z_REAR - 4, 0), (F.Z_REAR - 4, A_SET - CD), "모터 축\n26.95", ha="left", va="center", off=(1.5, -6), fs=8)
    dim(ax, (F.Z_REAR - 20, -60), (F.Z_REAR - 20, 0), "배관 내경\nØ120", ha="right", va="center", off=(-1.5, 0))
    lab(ax, (0, A_SET - CD + 4), (36, 20), "웜 (모터 D축에 직접)\n웜휠과 축간 14")
    lab(ax, (F.M_FACE + 1.5, A_SET - CD - 4), (16, -38), "모터 고정판\n(+ 스러스트 와셔)")
    lab(ax, (-25, 20), (F.Z_REAR - 43, 76), f"스프링 C202 (설치 길이 {spring_len(A_SET):.1f})")
    lab(ax, (-40, F.SEAT_Y + F.SPACER / 2), (F.Z_REAR - 43, 69), f"스페이서 {F.SPACER:.0f} (내경 100용은 뺌)")
    lab(ax, (-25, F.ROD_TOP - 1), (6, 70), "스토퍼 (바깥 끝 = 바퀴 축 43.95)")
    lab(ax, (-70, F.TRI_RV - 1.5), (-112, 52), "삼각 외피 (유닛 중심선 위치,\n창 구간은 비어 있음)", ha="center")
    lab(ax, (-20, A_SET - CD + 14), (2, 76), "슬라이더 (연한 파랑: 지지판 B·C·모터 위 막대·반원 받침·귀)")
    lab(ax, (F.Z_FRONT + 1.5, 4), (F.Z_FRONT + 8, -8), "ToF\n(앞판, 미선정)")
    ax.set_xlim(F.Z_REAR - 45, F.Z_FRONT + 40); ax.set_ylim(-66, 80)
    ax = fig.add_axes([.46, .03, .53, .31]); ax.axis("off")
    t1 = ("■ 주요 치수\n"
          f"· 배관 내경 120 (시연) / 삼각 외피 최대 반지름 {F.TRI_RV:.0f} / 길이 {L_MOD:.1f}\n"
          "· 모터 FIT0522: 기어박스 23 + 모터 31 + 엔코더 9, 축 14.5 [도면]\n"
          "· 웜을 D축(장착면 +4.5~14.5)에 직접 → 슬리브·웜 축 없음\n"
          f"· 가이드봉 Ø4 × {F.ROD_TOP - F.ROD_Y0:.1f} (반지름 {F.ROD_Y0}~{F.ROD_TOP:.0f}), 바퀴당 2개, 모두 바퀴 뒤\n"
          f"· 바퀴 축 Ø4 × 약 {F.AXLE_X1 + 19.5:.0f}, MF74ZZ 2개 / 유닛 (지지판 B·C, 간격 15)\n"
          f"· 출력 웜기어 1:{F.WHEEL_Z} → 무부하 약 {math.pi * 38.1 * 100 / F.WHEEL_Z / 60:.1f} mm/s [계산]\n"
          f"\n■ 스프링 C202 × 2 / 바퀴 + 스페이서 {F.SPACER:.0f}\n"
          f"· 내경 120 설치: 길이 {spring_len(A_SET):.1f}, 바퀴당 약 {wheel_force(A_SET):.1f} N\n"
          f"· 이음부 Ø123.8: 길이 {spring_len(A_JOINT):.1f}, 약 {wheel_force(A_JOINT):.1f} N\n"
          f"· 안쪽으로 {A_SET-A_SOLID:.1f} 더 들어감 (그 이상은 스프링 한계)\n"
          "· 스페이서를 빼고 짧은 봉을 쓰면 내경 100~106 직관용")
    t2 = ("■ 계산 확인 (cad/v2/front_module_v2.py)\n"
          "· CAD 간섭: 행정 6위치 × 고정부, 이웃 유닛 36조합 → 겹침 없음\n"
          "· 엘보 (CAD 표면 점 3D 검사): 내경 120 엘보 통과\n"
          "   내경 100 엘보는 불통과 (슬라이더 모서리 재설계 필요)\n"
          "· 정적 계산. 실물 엘보 시험으로 확인 필요\n"
          "\n■ 확인·구매가 필요한 것\n"
          "· 구매: MF74ZZ ×6, 스러스트 와셔 ×3, 봉 끝 스토퍼 ×6\n"
          f"· 가이드봉 {F.ROD_TOP - F.ROD_Y0:.1f} (Ø4 샤프트 절단), 부싱 LM4UU 4×8×12\n"
          "· 웜·웜휠은 출력 (PLA, 그리스)\n"
          "· FIT0522 플러그 방향·길이, 나사 간격\n"
          "· C202 안지름 > 4 인지\n"
          "· MDD3A 2개·ESP32는 뒤 마디 (모터선 6가닥 × 3이 관절 통과)")
    ax.text(0, 1, t1, fontsize=10.3, va="top", linespacing=1.42); ax.text(.5, 1, t2, fontsize=10.3, va="top", linespacing=1.42)
    fig.text(.02, .015, "배치 검토용 도면 · 2026-11-23 · v2 · 치수는 cad/v2/front_module_v2.py 기준 · 플러그 공간은 가정값 · 이전안은 figures/v1/", fontsize=9, color="#555")
    for p in paths: fig.savefig(p)
    plt.close(fig)

# ======================= Figure 2: 서스펜션 유닛 =======================
def fig_unit(path):
    fig = plt.figure(figsize=(19, 11.5), dpi=105)
    fig.suptitle("서스펜션 바퀴 유닛 v2 (1개 단독, 내경 120 설정)  —  단위 mm", fontsize=17, fontweight="bold", y=.975)
    ax = fig.add_axes([.005, .06, .32, .86]); ax.set_aspect("equal"); ax.axis("off"); ax.set_title("① 앞에서 본 모습 (설치 상태)", fontsize=13, loc="left")
    th = np.linspace(np.radians(50), np.radians(130), 80); ax.plot(60 * np.cos(th), 60 * np.sin(th), "k", lw=1.4)
    ax.add_patch(Circle((0, 0), 7.5, fc="#6d7785", ec="k")); ax.add_patch(Circle((0, 0), 4, fc="w", ec="k", lw=.6)); unit_front(ax, A_SET)
    dim(ax, (-11.5, 66), (11.5, 66), "바퀴 폭 23"); dim(ax, (0, -14), (MX, -14), f"모터 옆으로 {MX:.0f}", va="top", off=(0, -1))
    ax.plot([MX, MX], [A_SET - CD, -16], "k", lw=.4, ls=":"); ax.plot([0, 0], [0, -16], "k", lw=.4, ls=":")
    dim(ax, (44, A_SET - CD), (44, A_SET), "축간 14", ha="left", va="center", off=(1.5, 0))
    ax.plot([MX, 46], [A_SET - CD] * 2, "k", lw=.4, ls=":"); ax.plot([MX + 2, 46], [A_SET] * 2, "k", lw=.4, ls=":")
    lab(ax, (-15.5, A_SET + 6), (-34, 54), "허브", ha="right"); lab(ax, (MX, A_SET + 7), (36, 58), "웜휠")
    lab(ax, (MX + 9, A_SET - CD - 6), (40, 8), "모터 Ø25\n(빗금 = 웜)"); lab(ax, (17, A_SET + 4), (8, 74), "지지판 B·C (베어링 각 1)", ha="center")
    lab(ax, (-4, A_SET - F.EAR_OFF + 4), (-34, 30), "귀 (부싱) + 팔", ha="right")
    lab(ax, (-3, 24), (-34, 21), "스프링 C202", ha="right"); lab(ax, (-3, 13), (-34, 12), f"스페이서 {F.SPACER:.0f}", ha="right")
    lab(ax, (-2, 6), (-34, 3), "받침 블록 (기둥)", ha="right"); ax.text(0, 70, "배관 벽 (내경 120)", ha="center", fontsize=9)
    ax.set_xlim(-50, 58); ax.set_ylim(-30, 80)
    ax = fig.add_axes([.33, .42, .41, .5]); ax.set_aspect("equal"); ax.axis("off"); ax.set_title("② 옆에서 본 모습 (설치 상태, 왼쪽이 뒤)", fontsize=13, loc="left")
    body_side(ax, x0=F.Z_REAR - 5, x1=F.Z_FRONT + 10); unit_side(ax, A_SET, cover=False)
    e = A_SET - F.EAR_OFF; xd = F.Z_REAR - 10
    for y in (F.ROD_Y0, F.SEAT_Y, SEAT, e, e + F.EAR_H, F.ROD_TOP): ax.plot([xd - 12, -44], [y, y], "k", lw=.4, ls=":", zorder=1)
    for y0, y1, t in ((F.ROD_Y0, F.SEAT_Y, f"블록 {F.SEAT_Y - F.ROD_Y0:.1f}"), (F.SEAT_Y, SEAT, f"스페이서 {F.SPACER:.0f}"), (SEAT, e, f"스프링 {spring_len(A_SET):.1f}"), (e, e + F.EAR_H, f"귀 {F.EAR_H:.0f}")):
        dim(ax, (xd, y0), (xd, y1), t, ha="left", va="center", off=(1, 0), fs=7.5)
    dim(ax, (xd - 10, F.ROD_Y0), (xd - 10, F.ROD_TOP), f"봉 {F.ROD_TOP - F.ROD_Y0:.1f}\n(반지름 {F.ROD_Y0}~{F.ROD_TOP:.0f})", ha="right", va="center", off=(-1, 0), fs=8)
    lab(ax, (-25, F.ROD_TOP - 1), (-8, 72), "스토퍼"); lab(ax, (F.Z_ENC + 4, A_SET - CD), (F.Z_ENC, 56), "엔코더", ha="center")
    lab(ax, (0, A_SET + WR - 3), (20, 72), "옴니휠 Ø38.1"); lab(ax, (2, A_SET - CD), (18, -16), "웜 (D축 직결)")
    lab(ax, (-30, A_SET - CD + 14.5), (-60, 64), "모터 위 막대 + 반원 받침", ha="center")
    ax.set_xlim(F.Z_REAR - 45, F.Z_FRONT + 20); ax.set_ylim(-26, 80)
    st = [(A_JOINT, "바깥쪽: 이음부 Ø123.8", 61.9), (A_SET, "설치: 내경 120", 60.0), (A_SOLID, "가장 눌린 상태 (스프링 한계)", A_SOLID + WR)]
    for i, (a, t, wall) in enumerate(st):
        ax = fig.add_axes([.33 + .137 * i, .06, .13, .32]); ax.set_aspect("equal"); ax.axis("off"); ax.set_title(t, fontsize=10.5)
        ax.add_patch(Rectangle((-48, wall), 66, 4, fc="#eef1f4", ec=C["pipe"])); ax.plot([-48, 18], [wall, wall], "k", lw=1.4)
        ax.add_patch(Rectangle((-48, -7.5), 66, 15, fc="#6d7785", ec="k", lw=.8)); unit_side(ax, a, cover=False)
        ax.text(-15, -12, f"바퀴 축 {a:.2f}\n스프링 {spring_len(a):.2f} · 바퀴당 약 {wheel_force(a):.1f} N", ha="center", va="top", fontsize=9)
        ax.set_xlim(-50, 20); ax.set_ylim(-24, 68)
    fig.text(.33, .395, "③ 스프링이 움직이는 범위 (모터는 일부만 보임)", fontsize=13)
    fig.text(.755, .9, "■ 유닛 1개의 부품\n\n구매품\n· 옴니휠 NX-14166 ×1\n· 모터 FIT0522 ×1\n"
             f"· 가이드봉 Ø4 ({F.ROD_TOP - F.ROD_Y0:.1f}) ×2, 끝 스토퍼 ×2\n· 리니어 볼부시 LM4UU ×2\n· 스프링 C202 ×2\n"
             f"· 바퀴 축 Ø4 약 {F.AXLE_X1 + 19.5:.0f}, MF74ZZ ×2\n· 스러스트 와셔 ×1\n\n"
             "3D 프린팅 (cad/v2/stl)\n· slider: 지지판 B·C·모터 고정판·위 막대·\n   반원 받침·귀 일체\n· hub (M2×4, PCD 12)\n· spacer Ø6.5 × 9 ×2\n· worm, wormwheel (모듈 1.0, 1:16)\n"
             "\n■ 움직이는 것 / 고정된 것\n· 움직임: 슬라이더·바퀴·웜기어·모터\n· 고정: 기둥·받침·가이드봉·스토퍼·덮개\n\n"
             "■ v1에서 바뀐 것\n· 슬리브·웜 축·웜 축 베어링 없음\n· 허브 쪽 지지판 없음 → 웜휠 바깥 지지판 C\n   (베어링 간격 15, 웜휠은 두 베어링 사이)\n· 가이드 2개 모두 바퀴 뒤 (v1: 앞뒤)\n"
             "· 모터 옆 오프셋 19 → 25\n· 바퀴 아래 바닥판 없음 (모터 위로 연결)\n\n"
             "■ 확인 필요\n· C202 안지름 > 4\n· 출력 웜기어의 마모·유격\n· 웜 하중을 모터 출력축이 견디는지",
             fontsize=10.1, va="top", linespacing=1.42)
    fig.text(.02, .015, "배치 검토용 도면 · 2026-11-23 · v2 · 치수는 cad/v2/front_module_v2.py 기준 · 이전안은 figures/v1/", fontsize=9, color="#555")
    fig.savefig(path); plt.close(fig)

if __name__ == "__main__":
    print(f"스프링: 설치 {spring_len(A_SET):.2f} / {wheel_force(A_SET):.1f} N, 이음부 {spring_len(A_JOINT):.2f} / {wheel_force(A_JOINT):.1f} N, 한계 바퀴 축 {A_SOLID:.2f}")
    fig_layout(sys.argv[1:3]); fig_unit(sys.argv[3]); print("saved", *sys.argv[1:4])
