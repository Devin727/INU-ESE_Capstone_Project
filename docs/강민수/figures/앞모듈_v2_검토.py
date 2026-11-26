"""앞 모듈 v2 엘보 통과 계산 (CAD 형상 기준, 3D 점 검사). 단위 mm.

실행: python 앞모듈_v2_검토.py            → ../cad/v2/elbow_results.txt, 2026-11-23-앞모듈_v2_엘보_계산.png  (cadquery 필요, 10분 이상)
      python 앞모듈_v2_검토.py --round-wheel → 둥근 바퀴 진단 결과를 elbow_results.txt 끝에 덧붙임
      python 앞모듈_v2_검토.py --closed-shell → 모터까지 덮는 닫힌 삼각 외피 진단 결과를 elbow_results.txt 끝에 덧붙임
방법:
- ../cad/v2/front_module_v2.py 의 실제 부품 형상(고정부 + 유닛 3개)을 삼각형으로 나눠 표면 점을 뽑는다.
  (z 2 mm × 축 둘레 5° 칸마다 가장 바깥 점만 남김)
- 엘보: 중심선이 한 평면에 있는 90° 굽힘. 'tube' = 중심선 반지름 Rc 굽힘, 'sharp' = 각진 모서리(두 직관 교차).
  점이 배관 안에 있는지는 3D로 판정한다 (중심선까지 거리 ≤ 내경/2).
- 로봇 축은 굽힘 평면 안에 둔다. 축 각도 0~90° (3° 간격)마다 들어가는 위치가 있는지 1 mm 격자로 찾는다.
  로봇이 자기 축 둘레로 돈 각(롤) 0~105° (15° 간격, 120° 대칭) 모두에서 성립해야 "통과".
- 유닛 3개는 그 설정에서 가장 안쪽 위치(스프링 한계)에 둔다. 바퀴는 Ø38.1 × 23 원통으로 본다 (실제 옴니휠보다 모서리가 큼).
한계: 정적 형상 검사. 각도 사이 연속성, 힘·마찰, 로봇 축이 굽힘 평면을 벗어나는 자세, 관절·뒤 모듈, 엘보 소켓 단차는 넣지 않음.
"""
import sys, math, pathlib
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cad" / "v2"))
import front_module_v2 as F

SHORT_ROD = 37.0     # 내경 100 설정용 짧은 가이드봉 끝 반지름 (그 설정의 귀 윗면 최대 35 + 스토퍼 2)
ELBOWS = [("내경 120, DV 125 엘보 근사 (Rc 75)", 120, 75, "tube"), ("내경 120, 각진 모서리", 120, 75, "sharp"),
          ("내경 100, DV 100 엘보 근사 (Rc 62)", 100, 62, "tube"), ("내경 100, 각진 모서리", 100, 62, "sharp")]
ROLLS = np.radians(np.arange(0, 120, 15)); THETAS = np.radians(np.arange(0, 90.1, 3))

def surface_points(wp, tol=0.3, step=3.0):
    out = []
    for s in wp.solids().vals():
        v, t = s.tessellate(tol, 0.5)
        V = np.array([(p.x, p.y, p.z) for p in v]); T = V[np.array(t)]
        out.append(V)
        L = np.max(np.linalg.norm(T - np.roll(T, 1, axis=1), axis=2), axis=1); n = np.ceil(L / step).astype(int)
        for k in np.unique(n[n > 1]):
            tri = T[n == k]; ij = [(i / k, j / k) for i in range(k + 1) for j in range(k + 1 - i)]
            b = np.array([(1 - i - j, i, j) for i, j in ij])
            out.append(np.einsum("pb,tbc->tpc", b, tri).reshape(-1, 3))
    return np.concatenate(out)
def robot_cloud(d, rod_top):
    """고정부 + 유닛 3개(변위 d)의 가장 바깥 표면 점."""
    P = {n: f() for n, f in F.PRINT.items() if n not in ("worm", "wormwheel")}
    U = F.unit_parts(P, F.bought_moving())
    old = F.ROD_TOP; F.ROD_TOP = rod_top; rods = F.rods_unit(); F.ROD_TOP = old
    parts = [P["base"], P["front_cap"], P["cover"]]
    for k in range(3): parts += [F.rotz(rods, 120 * k)] + list(F.place(U, k, d).values())
    pts = np.concatenate([surface_points(p) for p in parts])
    r = np.hypot(pts[:, 0], pts[:, 1]); zi = np.floor(pts[:, 2] / 2).astype(int); ai = np.floor(np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) / 5).astype(int)
    key = (zi - zi.min()) * 1000 + (ai + 100); order = np.lexsort((-r, key)); key_s = key[order]
    first = np.r_[True, key_s[1:] != key_s[:-1]]; keep = pts[order][first]
    return keep[np.argsort(-np.hypot(keep[:, 0], keep[:, 1]))]          # 바깥 점부터 (빨리 걸러지게)
def inside(px, py, pz, D, Rc, model):
    h2 = (D / 2) ** 2
    if model == "sharp":
        return (((px - Rc) ** 2 + pz ** 2 <= h2) & (py <= Rc + D / 2)) | (((py - Rc) ** 2 + pz ** 2 <= h2) & (px <= Rc + D / 2))
    d1 = np.where(py <= 0, np.abs(px - Rc), np.hypot(px - Rc, py))
    d2 = np.where(px <= 0, np.abs(py - Rc), np.hypot(px, py - Rc))
    phi = np.arctan2(py, px)
    d3 = np.where((phi >= 0) & (phi <= np.pi / 2), np.abs(np.hypot(px, py) - Rc), np.inf)
    return np.minimum(np.minimum(d1, d2), d3) ** 2 + pz ** 2 <= h2 + 1e-9
def fits(pts, D, Rc, model, th, roll, step=1.0, chunk=150):
    s = pts[:, 0] * np.cos(roll) + pts[:, 1] * np.sin(roll); w = -pts[:, 0] * np.sin(roll) + pts[:, 1] * np.cos(roll)
    u = (-np.sin(th), np.cos(th)); n = (np.cos(th), np.sin(th))
    lx = pts[:, 2] * u[0] + s * n[0]; ly = pts[:, 2] * u[1] + s * n[1]
    X, Y = np.meshgrid(np.arange(Rc - D, Rc + D / 2, step), np.arange(-D, Rc + D / 2, step)); X, Y = X.ravel(), Y.ravel()
    for i in range(0, lx.size, chunk):
        ok = inside(X[:, None] + lx[None, i:i + chunk], Y[:, None] + ly[None, i:i + chunk], w[None, i:i + chunk], D, Rc, model).all(axis=1)
        X, Y = X[ok], Y[ok]
        if X.size == 0: break
    return X, Y
def passes(pts, D, Rc, model):
    """모든 롤·축 각도에서 들어가는 위치가 있으면 (True, None). 아니면 (False, 처음 막힌 (롤°, 축 각°))."""
    for roll in ROLLS:
        for th in THETAS:
            if fits(pts, D, Rc, model, th, roll)[0].size == 0: return False, (round(math.degrees(roll)), round(math.degrees(th)))
    return True, None

CASES = [("내경 120 설정 (스페이서 9, 긴 봉). 유닛은 스프링 한계", F.D_SOLID, F.ROD_TOP, (0, 1)),
         ("스페이서 없음 + 짧은 봉. 유닛은 가장 안쪽", F.D_MIN, SHORT_ROD, (0, 1, 2, 3)),
         ("스페이서 없음 + 긴 봉. 유닛은 가장 안쪽", F.D_MIN, F.ROD_TOP, (2, 3))]

def draw(path, clouds, results):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager as fm
    for f in ("C:/Windows/Fonts/malgun.ttf", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
        try: fm.fontManager.addfont(f); plt.rcParams["font.family"] = fm.FontProperties(fname=f).get_name(); break
        except Exception: pass
    plt.rcParams["axes.unicode_minus"] = False
    fig = plt.figure(figsize=(18, 10.5), dpi=100)
    fig.suptitle("앞 모듈 v2 엘보 통과 계산 — CAD 형상 기준 3D 점 검사 (단위 mm)", fontsize=16, fontweight="bold", y=.98)
    for col, (ci, ei) in enumerate(((0, 0), (1, 2))):
        name, d, rod, _ = CASES[ci]; en, D, Rc, model = ELBOWS[ei]; pts = clouds[ci]
        ax = fig.add_axes([.02 + col * .49, .30, .47, .62]); ax.set_aspect("equal"); ax.axis("off")
        ax.set_title(f"{en}\n{name} (바퀴 축 {F.A0 + d:.2f}), 롤 0°", fontsize=11.5, loc="left")
        t = np.linspace(0, np.pi / 2, 100)
        for r in (Rc - D / 2, Rc + D / 2): ax.plot(r * np.cos(t), r * np.sin(t), "k", lw=1.3)
        for xx in (Rc - D / 2, Rc + D / 2): ax.plot([xx, xx], [0, -150], "k", lw=1.3); ax.plot([0, -150], [xx, xx], "k", lw=1.3)
        for th_deg, colr in ((0, "#3f6fb5"), (45, "#c9772a"), (90, "#2f9e6e")):
            th = math.radians(th_deg); X, Y = fits(pts, D, Rc, model, th, 0.0)
            if X.size == 0: continue
            j = np.argmin((X - X.mean()) ** 2 + (Y - Y.mean()) ** 2)
            if th_deg == 0: j = np.argmin(Y)
            if th_deg == 90: j = np.argmin(X)
            lx = pts[:, 2] * -math.sin(th) + pts[:, 0] * math.cos(th); ly = pts[:, 2] * math.cos(th) + pts[:, 0] * math.sin(th)
            ax.scatter(X[j] + lx, Y[j] + ly, s=1.2, color=colr, alpha=.6, label=f"축 각도 {th_deg}°")
        ax.legend(loc="lower left", fontsize=9, markerscale=6); ax.set_xlim(-150, Rc + D / 2 + 8); ax.set_ylim(-150, Rc + D / 2 + 8)
    ax = fig.add_axes([.03, .02, .94, .25]); ax.axis("off")
    lines = ["■ 결과 (모든 롤 각 0~105°, 축 각도 0~90°에서 들어가는 위치가 있으면 통과)"] + ["  " + r for r in results] + [
        "", "점 = 부품 표면의 가장 바깥 점(굽힘 평면에 투영). 바퀴는 Ø38.1 × 23 원통으로 봄. 정적 형상 검사이며 실물 엘보 시험으로 확인해야 함.",
        f"치수: cad/v2/front_module_v2.py (길이 {F.Z_FRONT - F.Z_REAR:.1f}, 삼각 외피 최대 반지름 {F.TRI_RV:.0f}, 긴 봉 끝 r {F.ROD_TOP:.0f}, 짧은 봉 끝 r {SHORT_ROD:.0f})"]
    ax.text(0, 1, "\n".join(lines), va="top", fontsize=10.5)
    fig.savefig(path, bbox_inches="tight"); print("saved", path)

def round_wheel_diag(out_txt):
    """진단: 바퀴 가장자리를 반지름 5로 둥글린 형상으로 내경 100 엘보만 다시 계산해 결과 파일 끝에 덧붙인다.
    실제 옴니휠(롤러가 둥긂)은 원통과 이 형상 사이쯤이다."""
    orig = F.bought_moving
    def patched():
        d = orig(); d["wheel"] = F.cylx(F.WR, -F.WW / 2, F.WW / 2, F.A0, 0).edges().fillet(5.0); return d
    F.bought_moving = patched
    pts = robot_cloud(F.D_MIN, SHORT_ROD); F.bought_moving = orig
    out = [f"[진단] 바퀴 가장자리를 반지름 5로 둥글림. 스페이서 없음 + 짧은 봉, 유닛은 가장 안쪽. 최대 반지름 {np.hypot(pts[:, 0], pts[:, 1]).max():.1f}"]
    for i in (2, 3):
        en, D, Rc, model = ELBOWS[i]; ok, where = passes(pts, D, Rc, model)
        out.append(f"[진단] 둥근 바퀴 | {en}: " + ("통과" if ok else f"불통과 (롤 {where[0]}°, 축 각도 {where[1]}°에서 막힘)")); print(out[-1], flush=True)
    with open(out_txt, "a", encoding="utf-8") as f: f.write("\n".join(out) + "\n")

CLOSED = ((30.0, 54.1, 58.0), (20.0, 52.0, 57.0))   # (법선 각, 평면 거리, 둥근 부분 반지름). 내경 120 설정에서 모터·지지판이 닿지 않는 거리 + 여유 1 + 벽 1.5 [계산]
def closed_shell_diag(out_txt):
    """진단: 덮개를 창 없는 닫힌 삼각 외피(모터까지 안에 넣는 크기)로 바꿔 내경 120 엘보만 계산해 결과 파일 끝에 덧붙인다.
    viewer.html 처럼 세 변을 평면으로 닫는 외피가 v2 속구조에서 가능한지 보려는 것."""
    orig = F.PRINT["cover"]; out = []
    for psi, d, rv in CLOSED:
        F.PRINT["cover"] = lambda: F.tri_prism(d, rv, F.Z_REAR, F.Z_FRONT, psi).cut(F.tri_prism(d - 1.5, rv - 1.5, F.Z_REAR - 1, F.Z_FRONT + 1, psi))
        name, dd, rod, _ = CASES[0]; pts = robot_cloud(dd, rod)
        for i in (0, 1):
            en, D, Rc, model = ELBOWS[i]; ok, where = passes(pts, D, Rc, model)
            out.append(f"[진단] 닫힌 삼각 외피 (법선 {psi:.0f}°, 평면 거리 {d}, 둥근 부분 {rv}), 내경 120 설정 | {en}: " + ("통과" if ok else f"불통과 (롤 {where[0]}°, 축 각도 {where[1]}°에서 막힘)")); print(out[-1], flush=True)
    F.PRINT["cover"] = orig
    with open(out_txt, "a", encoding="utf-8") as f: f.write("\n".join(out) + "\n")

if __name__ == "__main__":
    out_txt = HERE.parent / "cad" / "v2" / "elbow_results.txt"
    if "--round-wheel" in sys.argv:
        round_wheel_diag(out_txt); sys.exit(0)
    if "--closed-shell" in sys.argv:
        closed_shell_diag(out_txt); sys.exit(0)
    lines = [f"모듈 길이 {F.Z_FRONT - F.Z_REAR:.1f}, 삼각 외피 (평면 거리 {F.TRI_D:g}, 최대 반지름 {F.TRI_RV:g}), 가이드봉 끝 반지름 {F.ROD_TOP:.0f} (짧은 봉 {SHORT_ROD:.0f})",
             f"롤 {len(ROLLS)}개 (0~105°, 15° 간격) × 축 각도 {len(THETAS)}개 (0~90°, 3° 간격), 위치 격자 1 mm"]
    clouds, results = [], []
    for name, d, rod, idx in CASES:
        pts = robot_cloud(d, rod); clouds.append(pts)
        print(f"{name}: 점 {len(pts)}개, 최대 반지름 {np.hypot(pts[:, 0], pts[:, 1]).max():.1f}", flush=True)
        for i in idx:
            en, D, Rc, model = ELBOWS[i]; ok, where = passes(pts, D, Rc, model)
            r = f"{name} (바퀴 축 {F.A0 + d:.2f}) | {en}: " + ("통과" if ok else f"불통과 (롤 {where[0]}°, 축 각도 {where[1]}°에서 막힘)")
            results.append(r); print(r, flush=True)
    out_txt.write_text("\n".join(lines + results) + "\n", encoding="utf-8")
    draw(HERE / "2026-11-23-앞모듈_v2_엘보_계산.png", clouds, results)
