"""v2 CAD 설명용 그림 (회의·발표용). 실행: python 앞모듈_v2_설명그림_draw.py  (cadquery, matplotlib, Pillow, 맑은 고딕)
이 폴더에 2026-11-23-앞모듈_v2_한눈에.png, _전체구조.png, _바퀴유닛.png, _분해도.png 를 만든다.
../cad/v2/front_module_v2.py 의 실제 형상을 그린다. 확인용 그림이며 치수 도면이 아니다.
"""
import io, sys, pathlib
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, ImageDraw, ImageFont

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cad" / "v2"))
import front_module_v2 as F

OUT = HERE; PRE = "2026-11-23-앞모듈_v2_"
W, H = 1800, 1120
BG = "#F3F5F3"; INK = "#18372F"; MUTED = "#53675F"; RED = "#A33A2C"
C = dict(slider="#4386BA", hub="#E1B14C", wheel="#2F5F93", wormwheel="#D7A63A", worm="#C9922A", axle="#596C7B", bearing="#C66643",
         bushing="#7A8B99", motor="#B9C1C8", plug="#8FB79A", base="#466E5E", front_cap="#6E8F82", cover="#C9D3CE", rods="#3A4447", spacer="#B68CB8")
LIGHT = np.array([0.35, -0.55, 0.75]); LIGHT = LIGHT / np.linalg.norm(LIGHT)
FOOT = "INU 캡스톤  |  앞 모듈 기구 v2  |  2026.11.23  |  설계 검토용 · 실물 출력·조립 전"

def font(size, b=False): return ImageFont.truetype("C:/Windows/Fonts/malgun" + ("bd" if b else "") + ".ttf", size)
def text(d, x, y, s, size=27, b=False, color=INK): d.text((x, y), s, font=font(size, b), fill=color)
def page(title, sub):
    im = Image.new("RGB", (W, H), BG); d = ImageDraw.Draw(im)
    text(d, 65, 40, title, 49, True); text(d, 68, 112, sub, 24, color=MUTED)
    d.line((65, 170, 1735, 170), fill="#C5D3CA", width=2); text(d, 65, 1070, FOOT, 21, color=MUTED)
    return im, d
def tris(wp):
    out = []
    for s in wp.solids().vals():
        v, t = s.tessellate(0.25, 0.4); V = np.array([(p.x, p.y, p.z) for p in v]); out.append(V[np.array(t)])
    return np.concatenate(out)
def render(items, size=(1040, 765), view=(20, -55)):
    """items: [(이름, 형상, 색, 투명도)]. 반환: (PIL 그림, {이름: 화면 좌표})."""
    T, Cs, cen = [], [], {}
    for name, wp, color, alpha in items:
        tr = tris(wp); n = np.cross(tr[:, 1] - tr[:, 0], tr[:, 2] - tr[:, 0]); n /= np.linalg.norm(n, axis=1)[:, None] + 1e-12
        sh = 0.45 + 0.55 * np.abs(n @ LIGHT); rgb = np.clip(np.array(to_rgb(color))[None] * sh[:, None], 0, 1)
        T.append(tr); Cs.append(np.c_[rgb, np.full(len(tr), alpha)]); cen.setdefault(name, tr.reshape(-1, 3).mean(0))
    T = np.concatenate(T); Cs = np.concatenate(Cs)
    fig = plt.figure(figsize=(size[0] / 100, size[1] / 100), dpi=100); ax = fig.add_axes([0, 0, 1, 1], projection="3d")
    ax.add_collection3d(Poly3DCollection(T, facecolors=Cs, edgecolors="none"))
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2]); ax.set_box_aspect(hi - lo, zoom=1.15)
    ax.view_init(elev=view[0], azim=view[1]); ax.set_axis_off(); fig.canvas.draw()
    pix = {}
    for name, c in cen.items():
        x2, y2, _ = proj3d.proj_transform(c[0], c[1], c[2], ax.get_proj()); px, py = ax.transData.transform((x2, y2)); pix[name] = (px, size[1] - py)
    buf = io.BytesIO(); fig.savefig(buf, format="png", dpi=100, facecolor="white"); plt.close(fig); buf.seek(0)
    return Image.open(buf).convert("RGB"), pix
def legend(d, rows, x=1170, y=235, gap=160):
    for i, (color, head, l1, l2) in enumerate(rows):
        yy = y + gap * i; d.rounded_rectangle((x, yy + 4, x + 25, yy + 29), 5, fill=color)
        text(d, x + 41, yy - 6, head, 32, True); text(d, x + 41, yy + 44, l1, 25, color=MUTED); text(d, x + 41, yy + 78, l2, 25, color=MUTED)

P = {n: f() for n, f in F.PRINT.items()}
U = F.unit_parts(P, F.bought_moving()); U["worm"] = F.worm(); U["wormwheel"] = F.wormwheel()
rods = F.rods_unit()
spacers = [P["spacer"].translate((0, 0, gz - F.GUIDE_Z[0])) for gz in F.GUIDE_Z]

# 1) 전체 구조
items = [("base", P["base"], C["base"], 1), ("front_cap", P["front_cap"], C["front_cap"], 1)]
for k in range(3):
    items.append(("rods", F.rotz(rods, 120 * k), C["rods"], 1))
    items += [("spacer", F.rotz(s, 120 * k), C["spacer"], 1) for s in spacers]
    items += [(n, p, C[n], 1) for n, p in F.place(U, k, 0).items()]
items.append(("cover", P["cover"], C["cover"], 0.38))
im, d = page("앞 모듈 전체 구조 (v2)", "현재 CAD · 내경 120 설치 상태 · 덮개는 반투명으로 표시")
img, _ = render(items); im.paste(img, (120, 185))
legend(d, [(C["wheel"], "옴니휠 3개", "배관 둘레에 120° 간격", "스프링이 벽 쪽으로 밀어 붙임"),
           (C["slider"], "움직이는 바퀴 유닛", "슬라이더·모터·웜기어·바퀴가 함께 이동", "가이드봉 2개와 볼부시 LM4UU로 안내"),
           (C["base"], "고정부", "뒷판 + 가운데 기둥 + 앞판", "가이드봉과 스프링 받침을 지지"),
           (C["cover"], "덮개 Ø78", "한 덩어리, 유닛마다 창", "MDD3A·ESP32는 뒤 마디에 둠")])
text(d, 80, 968, f"시연 기준 배관 내경 120 mm  /  덮개 Ø78 mm  /  길이 {F.Z_FRONT - F.Z_REAR:.1f} mm", 30, True)
text(d, 80, 1014, "계산상 내경 120 엘보 통과 · 내경 100은 직관만 (엘보 불통과) · 실물 검증 전", 25, color=RED)
im.save(OUT / (PRE + "전체구조.png")); whole_img = img

# 2) 바퀴 유닛 구성
unit = [(n, p, C[n], 1) for n, p in F.place(U, 0, 0).items()] + [("rods", rods, C["rods"], 1)] + [("spacer", s, C["spacer"], 1) for s in spacers]
im, d = page("바퀴 유닛 하나의 구성 (v2)", "현재 CAD 형상 · 옴니휠 롤러는 원통 외형으로 표현 · 스프링은 그림에서 생략")
img, _ = render(unit, view=(18, -40)); im.paste(img, (120, 185))
legend(d, [(C["motor"], "모터 FIT0522", "배관 길이 방향으로 배치, 엔코더 내장", "출력축에 웜을 직접 끼움"),
           (C["wormwheel"], "출력 웜기어 1:16", "모듈 1.0, 한 줄 웜 + 웜휠 16T", f"무부하 약 {np.pi * 38.1 * 100 / F.WHEEL_Z / 60:.1f} mm/s"),
           (C["bearing"], "바퀴 축 베어링 2개", "플랜지형 MF74ZZ, 간격 15 mm", "웜휠이 두 베어링 사이에 있음"),
           (C["spacer"], "서스펜션", "가이드봉 Ø4 + LM4UU + 스프링 C202", "스페이서 9 mm = 내경 120 설정")])
text(d, 80, 968, "조립 흐름  ·  모터 체결 → 웜 → 베어링·웜휠 → 허브·바퀴와 축 → 봉에 끼우고 스토퍼", 28, True)
text(d, 80, 1014, "웜기어 수명·정지 시 잠김, 모터 나사 간격, 플러그 공간은 실물 확인 필요", 25, color=RED)
im.save(OUT / (PRE + "바퀴유닛.png")); unit_img = img

# 3) 분해도
OFF = dict(motor=(0, 0, -34), plug=(0, 0, -34), worm=(0, -24, 0), wormwheel=(0, 0, 26), bearing=(0, 20, 0), axle=(0, 36, 0),
           wheel=(-30, 0, 0), hub=(-50, 0, 0), bushing=(-22, 0, 0))
ex = [(n, p.translate(OFF.get(n, (0, 0, 0))), C[n], 1) for n, p in F.place(U, 0, 0).items()]
ex += [("rods", rods.translate((0, -16, 0)), C["rods"], 1)] + [("spacer", s.translate((0, -16, 0)), C["spacer"], 1) for s in spacers]
W2, H2 = 2000, 1320
im = Image.new("RGB", (W2, H2), BG); d = ImageDraw.Draw(im)
text(d, 60, 36, "바퀴 유닛 · 3D 분해도 (v2)", 49, True)
text(d, 62, 104, "front_module_v2.py의 실제 CAD 형상으로 렌더링  ·  부품 위치를 벌려 표시 · 벌린 간격은 설명용", 24, color=MUTED)
img, pix = render(ex, size=(1060, 1060), view=(20, -50)); im.paste(img, (178, 155))
labels = [("motor", "FIT0522 모터", "배관 길이 방향, 뒤에 엔코더"), ("slider", "슬라이더 (출력)", "지지판 B·C, 모터 고정판, 귀 일체"),
          ("wheel", "옴니휠", "CAD에서는 원통 외형으로 표현"), ("hub", "허브 (출력)", "바퀴와 Ø4 mm 축 연결"),
          ("bearing", "MF74ZZ 베어링 2개", "지지판 B·C에서 바퀴 축 지지"), ("wormwheel", "웜휠 (출력, 16T)", "바퀴 축에 달리는 기어"),
          ("worm", "웜 (출력)", "모터 D축에 직접 끼움"), ("bushing", "LM4UU 볼부시 2개", "귀에 끼워 가이드봉을 탐"),
          ("rods", "가이드봉 · 스페이서", "기둥에 고정, 끝에 스토퍼")]
for i, (key, head, sub) in enumerate(labels):
    y = 200 + i * 106; px, py = pix[key]; px += 178; py += 155; col = C[key] if key != "rods" else "#3A4447"
    d.line((px, py, 1360, y + 18, 1425, y + 18), fill=col, width=2); d.ellipse((px - 5, py - 5, px + 5, py + 5), fill=col, outline="white")
    text(d, 1445, y - 4, head, 30, True, color=col); text(d, 1445, y + 40, sub, 23)
text(d, 60, 1190, "남은 확인 사항  ·  출력 웜기어 마모·유격 / 바퀴 한쪽 지지의 휨 / 가이드봉 끝 스토퍼 고정 방법", 27, True, color=RED)
text(d, 60, 1236, "스프링 C202는 그림에서 생략  ·  구매품은 CAD의 단순 외형으로 표시", 24, color=MUTED)
text(d, 60, 1278, FOOT, 21, color=MUTED)
im.save(OUT / (PRE + "분해도.png")); ex_img = img

# 4) 한눈에 보기 (한 장 요약)
W3, H3 = 2400, 1400; GREEN = "#1F7A4D"
im = Image.new("RGB", (W3, H3), BG); d = ImageDraw.Draw(im)
text(d, 60, 34, "앞 모듈 v2 한눈에 보기", 52, True)
text(d, 62, 106, "배관 안을 바퀴 3개로 밀착해 주행하는 앞 마디  ·  내경 120 시연 기준  ·  현재 CAD 형상", 25, color=MUTED)
d.line((60, 160, W3 - 60, 160), fill="#C5D3CA", width=2)
def panel(img, x, y, w, cap):
    h = int(img.height * w / img.width); im.paste(img.resize((w, h), Image.LANCZOS), (x, y))
    d.rectangle((x, y, x + int(d.textlength(cap, font=font(25, True))) + 28, y + 50), fill="white"); text(d, x + 14, y + 10, cap, 25, True); return y + h
yb = panel(whole_img, 60, 185, 800, "① 전체 구조 (덮개 반투명)")
panel(unit_img, 880, 185, 800, "② 바퀴 유닛 1개 (×3, 120° 간격)")
def box(x0, y0, x1, y1, title, lines, col=INK):
    d.rounded_rectangle((x0, y0, x1, y1), 14, fill="white", outline="#C5D3CA", width=2)
    text(d, x0 + 22, y0 + 14, title, 29, True, color=col)
    for i, ln in enumerate(lines):
        c = INK
        if isinstance(ln, tuple): ln, c = ln
        text(d, x0 + 22, y0 + 62 + i * 37, ln, 24, color=c)
y0 = yb + 25
box(60, y0, 860, y0 + 250, "움직이는 방식", ["모터 FIT0522 → 웜(모터 축에 직결) → 웜휠 → 바퀴 축 → 옴니휠",
    f"출력 웜기어 감속 1:{F.WHEEL_Z}  ·  무부하 약 {np.pi * 38.1 * 100 / F.WHEEL_Z / 60:.1f} mm/s",
    "모터가 멈추면 웜이 바퀴를 잡아 주는 것을 기대 (실물 확인 필요)", "모터 드라이버 MDD3A ×2 · ESP32는 뒤 마디에 둠"])
box(880, y0, 1680, y0 + 250, "벽에 붙는 방식", ["유닛(슬라이더·모터·바퀴)이 가이드봉 2개를 타고 반지름 방향으로 이동",
    "스프링 C202 ×2가 바퀴를 벽으로 밀어 줌 (볼부시 LM4UU로 안내)", "내경 120: 바퀴당 약 11 N  ·  이음부 Ø123.8: 약 6 N [계산]",
    "스페이서 9 mm = 내경 120 설정. 빼면 내경 100~106 직관용"])
y1 = y0 + 275
box(60, y1, 860, y1 + 215, "남은 일", ["구매: MF74ZZ 베어링 ×6, 와셔, 시연용 125A 관·엘보",
    "출력: 슬라이더 1개 + 웜기어 1세트 + 기둥부터 맞춤 확인", "실측: 모터 나사 간격·플러그, 부싱 바깥지름, 스프링 안지름"])
box(880, y1, 1680, y1 + 215, "주의", [("실물 출력·조립·주행 시험은 아직 하지 않음 (이 장의 값은 모두 계산)", RED),
    "바퀴 축을 한쪽에서 받침 → 출력 후 휘는지 확인", "출력 웜기어의 마모·유격, 모터 축이 받는 웜 하중 확인"], col=RED)
xr = 1720
box(xr, 185, W3 - 60, 585, "크기와 구성", [f"덮개 Ø{2 * F.COV_RO:.0f} mm  ·  길이 {F.Z_FRONT - F.Z_REAR:.1f} mm", "옴니휠 Ø38.1 mm ×3 (Nexus 14166)",
    "모터 FIT0522 ×3 (6 V, 100 rpm, 엔코더)", f"가이드봉 Ø4 × {F.ROD_TOP - F.ROD_Y0:.1f} mm ×6", "바퀴 축 베어링 MF74ZZ ×6",
    "출력 부품: 슬라이더·허브·웜·웜휠 ×3,", "  스페이서 ×6, 기둥(뒷판 일체), 앞판, 덮개", "앞 모듈 무게 약 0.5 kg [추정]"])
box(xr, 610, W3 - 60, 1010, "계산 결과", [("○ 부품 간섭 없음 (행정 6위치, 유닛 쌍 36조합)", GREEN), ("○ 내경 120 엘보 통과 (두 모델)", GREEN),
    ("○ 누르는 힘: 수직 등반 필요값(5~8 N) 범위", GREEN), ("○ 모터 토크 여유 큼", GREEN), ("× 내경 100 엘보 불통과 → 직관만", RED),
    ("△ 모터 전원 6 V 2.7 A는 빠듯함", "#9C650D"), "정적 계산이며 실물 엘보 시험 필요"])
box(xr, 1035, W3 - 60, y1 + 215, "더 자세한 그림", ["분해도: 앞모듈_v2_분해도.png", "치수: 앞모듈_배치도_v2.png", "스프링 범위: 서스펜션_유닛_v2.png",
    "엘보: 앞모듈_v2_엘보_계산.png"])
text(d, 60, H3 - 48, FOOT + "  |  치수는 cad/v2/front_module_v2.py 기준", 21, color=MUTED)
im.save(OUT / (PRE + "한눈에.png"))
print("saved", *[p.name for p in sorted(OUT.glob(PRE + "*.png"))])
