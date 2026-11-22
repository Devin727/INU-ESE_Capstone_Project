"""v2 CAD 미리보기 그림 (matplotlib 3D, 단순 음영). 실행: python preview_v2.py  → preview/*.png
front_module_v2.py 의 형상을 그대로 삼각형으로 나눠 그린다. 확인용 그림이며 치수 도면이 아니다.
"""
import pathlib
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import font_manager as fm
import front_module_v2 as F

for f in ("C:/Windows/Fonts/malgun.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
    try: fm.fontManager.addfont(f); plt.rcParams["font.family"] = fm.FontProperties(fname=f).get_name(); break
    except Exception: pass
OUT = pathlib.Path(__file__).resolve().parent / "preview"; OUT.mkdir(exist_ok=True)
LIGHT = np.array([0.35, -0.55, 0.75]); LIGHT = LIGHT / np.linalg.norm(LIGHT)
COL = dict(slider="#7fb3e6", hub="#e2d6a8", wheel="#3f6fb5", wormwheel="#c9a227", worm="#c9a227", axle="#555555", bearing="#888888",
           bushing="#999999", motor="#b9c0c9", plug="#8fd19e", base="#6d7785", front_cap="#a9b1bb", petal="#d0d5db", rods="#333333", spacer="#c8a2c8")

def tris(wp, tol=0.25):
    out = []
    for s in wp.solids().vals():
        v, t = s.tessellate(tol, 0.4)
        V = np.array([(p.x, p.y, p.z) for p in v]); out.append(V[np.array(t)])
    return np.concatenate(out)
def render(items, path, title, elev=22, azim=-58, size=(9, 7.5)):
    T, C = [], []
    for wp, color, alpha in items:
        tr = tris(wp); n = np.cross(tr[:, 1] - tr[:, 0], tr[:, 2] - tr[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
        sh = 0.45 + 0.55 * np.abs(n @ LIGHT); rgb = np.clip(np.array(to_rgb(color))[None] * sh[:, None], 0, 1)
        T.append(tr); C.append(np.c_[rgb, np.full(len(tr), alpha)])
    T = np.concatenate(T); C = np.concatenate(C)
    fig = plt.figure(figsize=size, dpi=120); ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolors="none"))
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2]); ax.set_box_aspect(hi - lo)
    ax.view_init(elev=elev, azim=azim); ax.set_axis_off(); ax.set_title(title, fontsize=12)
    fig.savefig(path, bbox_inches="tight"); plt.close(fig); print("saved", path)

if __name__ == "__main__":
    P = {n: f() for n, f in F.PRINT.items()}; mv = F.bought_moving()
    U = F.unit_parts(P, mv); U["worm"] = F.worm(); U["wormwheel"] = F.wormwheel()     # 실제 이 형상
    render([(U["worm"], COL["worm"], 1), (U["wormwheel"], "#d8b84a", 1)], OUT / "웜기어.png",
           f"출력 웜기어 (모듈 {F.GEAR_M:.1f}, 1:{F.WHEEL_Z}, 축간 {F.CD:.0f})", elev=20, azim=-50, size=(7, 6))
    for old in ("덮개_조각.png",):                                  # 이전 3조각 덮개 그림 정리
        if (OUT / old).exists(): (OUT / old).unlink()
    inner = [(P["base"], COL["base"], 1)]
    for k in range(3):
        inner.append((F.rotz(F.rods_unit(), 120 * k), COL["rods"], 1))
        for gz in F.GUIDE_Z: inner.append((F.rotz(P["spacer"].translate((0, 0, gz - F.GUIDE_Z[0])), 120 * k), COL["spacer"], 1))
        for n, p in F.place(U, k, 0).items(): inner.append((p, COL[n], 1))
    render(inner, OUT / "조립_속구조.png", "앞 모듈 v2 — 덮개·앞판 뺀 속구조 (내경 120 설치 상태)")
    render(inner + [(P["front_cap"], COL["front_cap"], 1), (P["cover"], COL["petal"], .55)], OUT / "조립_전체.png",
           "앞 모듈 v2 — 전체 (덮개 반투명)")
    unit = [(p, COL[n], 1) for n, p in F.place(U, 0, 0).items()] + [(F.rods_unit(), COL["rods"], 1)]
    unit += [(P["spacer"].translate((0, 0, gz - F.GUIDE_Z[0])), COL["spacer"], 1) for gz in F.GUIDE_Z]
    render(unit, OUT / "유닛.png", "바퀴 유닛 1개 (가이드봉·스페이서 포함)", elev=18, azim=-35)
    render([(P["slider"], COL["slider"], 1)], OUT / "슬라이더.png", "슬라이더 (출력 부품)", elev=25, azim=-40)
    render([(P["cover"], COL["petal"], 1)], OUT / "덮개.png", "덮개 (한 덩어리)", elev=15, azim=-70)
