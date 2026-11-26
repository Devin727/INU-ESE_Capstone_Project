"""앞 모듈 v2 삼각 외피 후보 비교 그림. 단위 mm.
실행: python 앞모듈_v2_삼각외피_draw.py → 2026-11-24-앞모듈_v2_삼각외피_검토.png  (cadquery 필요, 1~2분)
왼쪽 그림은 viewer.html (2026-11-24 판, 커밋되지 않은 파일, 2026-11-26 삭제) 에서 뽑아 둔 단면 선분
앞모듈_v2_삼각외피_viewer단면.json 을 읽어 그린다.
"""
import sys, math, json, pathlib
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
from matplotlib.transforms import Affine2D
from matplotlib.colors import to_rgb
from matplotlib import font_manager as fm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cad" / "v2"))
import front_module_v2 as F
for f in ("C:/Windows/Fonts/malgun.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
    try: fm.fontManager.addfont(f); plt.rcParams["font.family"] = fm.FontProperties(fname=f).get_name(); break
    except Exception: pass
plt.rcParams["axes.unicode_minus"] = False
CLOSED = (30.0, 54.1, 58.0)        # 모터까지 덮는 닫힌 삼각 외피 (앞모듈_v2_검토.py --closed-shell 진단 형상과 같은 값)

def outline(psi, d, rv, n=720):
    a = np.radians(np.linspace(0, 360, n, endpoint=False)); r = np.full(n, float(rv))
    for k in range(3):
        c = np.cos(a - math.radians(psi + 120 * k)); m = c > 1e-6; r[m] = np.minimum(r[m], d / c[m])
    return np.c_[r * np.cos(a), r * np.sin(a)]
def unit_patches(ax, k, d, **kw):
    tr = Affine2D().rotate_deg(120 * k) + ax.transData
    for p in F.cutter_prims():
        t = p[0]
        if t == "box": _, x0, x1, y0, y1, z0, z1 = p; ax.add_patch(Rectangle((x0, y0 + d), x1 - x0, y1 - y0, transform=tr, **kw))
        elif t == "cylz": _, r, z0, z1, cx, cy = p; ax.add_patch(Circle((cx, cy + d), r, transform=tr, **kw))
        elif t == "cylx": _, r, x0, x1, cy, cz = p; ax.add_patch(Rectangle((x0, cy + d - r), x1 - x0, 2 * r, transform=tr, **kw))
    for gz in F.GUIDE_Z: ax.add_patch(Rectangle((-2, F.ROD_Y0), 4, F.ROD_TOP - F.ROD_Y0, transform=tr, **kw))
def circle(ax, R, **kw):
    th = np.linspace(0, 2 * np.pi, 361); ax.plot(R * np.cos(th), R * np.sin(th), **kw)
def viewer_section(ax):
    sec = json.loads((HERE / "앞모듈_v2_삼각외피_viewer단면.json").read_text(encoding="utf-8"))["sections"]
    for (key, x0, segs), (c, lw) in zip(sec, (("#b0308a", 2.2), ("#b0308a", 2.2), ("#c9772a", 1.2), ("#3f6fb5", .8))):
        for xa, ya, xb, yb in segs: ax.plot([xa, xb], [ya, yb], c=c, lw=lw, solid_capstyle="round")
    circle(ax, 50, c="k", lw=1.2)
def render(ax, items, elev=18, azim=-62):
    light = np.array([0.35, -0.55, 0.75]); light /= np.linalg.norm(light); TT, CC = [], []
    for wp, color, alpha in items:
        for s in wp.solids().vals():
            v, t = s.tessellate(0.25, 0.4); tr = np.array([(p.x, p.y, p.z) for p in v])[np.array(t)]
            n = np.cross(tr[:, 1] - tr[:, 0], tr[:, 2] - tr[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
            rgb = np.clip(np.array(to_rgb(color))[None] * (0.45 + 0.55 * np.abs(n @ light))[:, None], 0, 1)
            TT.append(tr); CC.append(np.c_[rgb, np.full(len(tr), alpha)])
    TT = np.concatenate(TT); CC = np.concatenate(CC)
    ax.add_collection3d(Poly3DCollection(TT, facecolors=CC, edgecolors="none"))
    lo, hi = TT.reshape(-1, 3).min(0), TT.reshape(-1, 3).max(0)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2]); ax.set_box_aspect(hi - lo); ax.view_init(elev=elev, azim=azim); ax.set_axis_off()

if __name__ == "__main__":
    fig = plt.figure(figsize=(21, 12.2), dpi=100)
    fig.suptitle("앞 모듈 v2 삼각 외피 검토 (단위 mm, 계산·CAD 결과, 실물 미확인)", fontsize=19, fontweight="bold", y=.985)
    # ① viewer
    ax = fig.add_axes([.01, .33, .28, .58]); ax.set_aspect("equal"); ax.axis("off"); viewer_section(ax)
    ax.set_xlim(-64, 64); ax.set_ylim(-64, 64)
    ax.set_title("① viewer.html 의 삼각 몸체 (가정 내경 100)", fontsize=14, loc="left")
    ax.text(0, -60, "모터(주황)가 바퀴 뒤, 꼭짓점 안쪽에 있다.\n세 변이 비어 있어 평면으로 닫을 수 있다.", ha="center", va="top", fontsize=12)
    # ② v2 단면
    ax = fig.add_axes([.305, .33, .30, .58]); ax.set_aspect("equal"); ax.axis("off")
    for k in range(3): unit_patches(ax, k, 0, fc="#7fb3e6", ec="none", alpha=.35)
    circle(ax, 60, c="k", lw=1.2); circle(ax, 50, c="gray", lw=.8, ls="--"); circle(ax, F.COV_RO, c="#2f9e6e", lw=2)
    ax.add_patch(Polygon(outline(F.TRI_PSI, F.TRI_D, F.TRI_RV), closed=True, fc="none", ec="#b0308a", lw=2.4))
    ax.add_patch(Polygon(outline(*CLOSED), closed=True, fc="none", ec="#c33", lw=1.6, ls=(0, (5, 3))))
    ax.set_xlim(-64, 64); ax.set_ylim(-64, 64)
    ax.set_title("② 우리 v2 단면 (내경 120 설치 상태, 전 길이를 겹쳐 그림)", fontsize=14, loc="left")
    ax.plot([], [], c="#2f9e6e", lw=2, label=f"이전 원통 덮개 Ø{2 * F.COV_RO:.0f}")
    ax.plot([], [], c="#b0308a", lw=2.4, label=f"삼각 외피 (평면 거리 {F.TRI_D:g}, 꼭짓점 반지름 {F.TRI_RV:g})")
    ax.plot([], [], c="#c33", lw=1.6, ls=(0, (5, 3)), label=f"모터까지 덮으려면 필요한 크기 (평면 거리 {CLOSED[1]})")
    ax.plot([], [], c="k", lw=1.2, label="배관 내경 120"); ax.plot([], [], c="gray", lw=.8, ls="--", label="배관 내경 100")
    ax.legend(loc="upper center", bbox_to_anchor=(.5, .0), fontsize=11, frameon=False)
    # ③ 3D
    P = {n: f() for n, f in F.PRINT.items()}; mv = F.bought_moving(); U = F.unit_parts(P, mv); tri = P["cover"]; rnd = F.cover_round()
    col = dict(slider="#7fb3e6", hub="#e2d6a8", wheel="#3f6fb5", wormwheel="#c9a227", worm="#c9a227", axle="#555555", bearing="#888888", bushing="#999999", motor="#b9c0c9", plug="#8fd19e")
    def assembly(cover, color):
        items = [(P["base"], "#6d7785", 1), (P["front_cap"], "#a9b1bb", 1), (cover, color, 1)]
        for k in range(3):
            items.append((F.rotz(F.rods_unit(), 120 * k), "#333333", 1))
            for n, p in F.place(U, k, 0).items(): items.append((p, col[n], 1))
        return items
    for i, (cover, color, name) in enumerate(((rnd, "#c9ced6", "이전 원통 덮개"), (tri, "#d9a3c6", "삼각 외피"))):
        ax = fig.add_axes([.615 + i * .19, .52, .20, .40], projection="3d"); render(ax, assembly(cover, color)); ax.set_title(f"③ {name} (조립)" if i == 0 else f"{name} (조립)", fontsize=13, y=.97)
        ax = fig.add_axes([.615 + i * .19, .20, .20, .31], projection="3d"); render(ax, [(cover, color, 1)]); ax.set_title(f"{name} (단품)", fontsize=12, y=1.02)
    # 아래 설명
    ax = fig.add_axes([.02, .01, .96, .20]); ax.axis("off")
    vol_t = sum(s.Volume() for s in tri.solids().vals()) / 1000; vol_c = sum(s.Volume() for s in rnd.solids().vals()) / 1000
    res = HERE.parent / "cad" / "v2" / "elbow_results.txt"
    rl = [l for l in res.read_text(encoding="utf-8").split("\n") if "|" in l] if res.exists() else []
    def stat(keep):
        v = ["불통과" not in l for l in rl if keep(l)]
        return "결과 없음" if not v else "모두 통과" if all(v) else "모두 불통과" if not any(v) else f"{sum(v)}/{len(v)} 통과"
    el = ["■ 엘보 계산 (삼각 외피, 3D 점 검사)",
          "· 내경 120 엘보: " + stat(lambda l: "진단" not in l and "| 내경 120," in l),
          "· 내경 100 엘보: " + stat(lambda l: "진단" not in l and "| 내경 100," in l),
          "· [진단] 바퀴를 둥글게 본 내경 100 엘보: " + stat(lambda l: "둥근 바퀴 |" in l),
          "· [진단] 모터까지 덮는 닫힌 삼각 외피, 내경 120 엘보: " + stat(lambda l: "닫힌 삼각" in l),
          "· 원통 덮개에서는 둥근 바퀴 진단의 DV 100 근사 모델만 통과했다.",
          "· 자세한 값: cad/v2/elbow_results.txt"]
    def prof(c):                                     # 길이 방향 1 mm 간격 단면적 (1 mm 조각의 부피)
        return [sum(s.Volume() for s in c.intersect(F.box(-80, 80, -80, 80, z, z + 1)).solids().vals()) for z in range(int(F.Z_REAR) + 1, int(F.Z_FRONT))]
    pr, pt = prof(rnd), prof(tri)
    left = ["■ 결론",
            "· viewer 처럼 세 변을 평면으로 닫는 외피는 지금 v2 속구조로는 만들 수 없다.",
            "  모터가 바퀴 옆(접선 25)에 있어 세 변 자리로 튀어나온다.",
            f"· 모터까지 덮으려면 평면이 중심에서 {CLOSED[1]} 에 있어야 한다 (빨간 점선). 내경 120 관(반지름 60)을 거의 채운다.",
            "· 그래서 삼각 외피는 뼈대형이다. 꼭짓점을 유닛 사이 빈 자리에 두어 레일 세 줄로 앞뒤를 잇고, 평면 쪽은 유닛이 드나드는 창이다.",
            "  창은 모서리를 둥글린 반듯한 모양으로 냈다.",
            f"· 원통 덮개와 비교: 부피 {vol_c:.1f} → {vol_t:.1f} cm3, 최대 반지름 {F.COV_RO:.0f} → {F.TRI_RV:.0f}. 앞·뒷판과 나사 자리는 같다. 간섭 없음 [CAD 계산].",
            f"· 길이 방향 단면적: 최소 {min(pr):.0f} → {min(pt):.0f} mm2, 400 mm2 에 못 미치는 구간 {sum(a < 400 for a in pr)} → {sum(a < 400 for a in pt)} mm [CAD 계산, 강도 시험 아님]."]
    ax.text(0, 1, "\n".join(left), va="top", fontsize=12, linespacing=1.5)
    ax.text(.62, 1, "\n".join(el), va="top", fontsize=12, linespacing=1.5)
    out = HERE / "2026-11-24-앞모듈_v2_삼각외피_검토.png"; fig.savefig(out); print("saved", out)
