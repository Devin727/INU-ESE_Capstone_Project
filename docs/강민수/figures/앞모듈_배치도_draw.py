import numpy as np, math, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle
from matplotlib import font_manager as fm
fm.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams["font.family"]=fm.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name(); plt.rcParams["axes.unicode_minus"]=False
def R(deg): t=math.radians(deg); c,s=math.cos(t),math.sin(t); return np.array([[c,-s],[s,c]])
def rect(x0,x1,y0,y1): return np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
C=dict(pipe="#9aa7b4",body="#d9dee4",col="#8c96a3",wheel="#3f6fb5",motor="#b9c0c9",spring="#e07b1a",rod="#555",ear="#2f9e6e",gear="#c9a227",pi="#c2185b",plate="#7fb3e6")
def dim(ax,p,q,txt,off=(0,0),ha="center",va="bottom",fs=8.5):
    ax.annotate("",xy=p,xytext=q,arrowprops=dict(arrowstyle="<->",lw=0.8,color="k"))
    ax.text((p[0]+q[0])/2+off[0],(p[1]+q[1])/2+off[1],txt,ha=ha,va=va,fontsize=fs,bbox=dict(fc="w",ec="none",pad=0.6,alpha=.85))
def lab(ax,xy,xt,txt,fs=8.5,ha="left"):
    ax.annotate(txt,xy=xy,xytext=xt,fontsize=fs,ha=ha,va="center",arrowprops=dict(arrowstyle="-",lw=0.7,color="#333"))
fig=plt.figure(figsize=(19,11.5),dpi=110)
fig.suptitle("앞 모듈 배치도 (배관 내경 120, 라즈베리파이 Zero 2 W)  —  단위 mm",fontsize=17,fontweight="bold",y=0.975)
# ---------- front view ----------
ax=fig.add_axes([0.02,0.07,0.44,0.84]); ax.set_aspect("equal"); ax.axis("off")
ax.set_title("① 정면 단면 (앞에서 본 모습, 바퀴가 벽에 닿은 상태)",fontsize=13,loc="left")
ax.add_patch(Circle((0,0),67,fc="#eef1f4",ec=C["pipe"],lw=1)); ax.add_patch(Circle((0,0),60,fc="w",ec="k",lw=1.4))
ax.add_patch(Circle((0,0),50,fc=C["body"],ec="k",lw=1.2)); ax.add_patch(Circle((0,0),48,fc="w",ec="k",lw=0.6))
for k in range(3):
    M=R(120*k)
    def P(a): return a@M.T
    # window gap in body (white wedge) 
    ax.add_patch(Polygon(P(rect(-24,21,46.5,51.5)),fc="w",ec="none",zorder=2))
    ax.add_patch(Polygon(P(rect(-11.5,11.5,21.9,60)),fc=C["wheel"],ec="k",lw=1,alpha=.85,zorder=5))
    ax.add_patch(Polygon(P(rect(-19.5,-11.5,31.95,49.95)),fc="#e8e0c8",ec="k",lw=.7,zorder=5))   # hub
    ax.add_patch(Polygon(P(rect(-22.5,-20,34,44.5)),fc=C["plate"],ec="k",lw=.7,zorder=5))
    ax.add_patch(Polygon(P(rect(12,14.5,20,44.5)),fc=C["plate"],ec="k",lw=.7,zorder=5))
    ax.add_patch(Polygon(P(rect(15,19,32.95,48.95)),fc=C["gear"],ec="k",lw=.7,zorder=5))        # worm wheel PD16
    ax.plot(*P(np.array([[-24,40.95],[20.5,40.95]])).T,color="k",lw=.8,ls="-.",zorder=6)        # axle
    mc=M@np.array([17,26.95]); ax.add_patch(Circle(mc,12.5,fc=C["motor"],ec="k",lw=1,ls="--",alpha=.75,zorder=4))
    ax.add_patch(Circle(mc,2,fc="k",zorder=6))
    for xr in (6,-6):
        ax.add_patch(Polygon(P(rect(xr-2,xr+2,3,34)),fc=C["rod"],ec="k",lw=.5,zorder=7))
        ax.add_patch(Polygon(P(rect(xr-3.25,xr+3.25,9,21.5)),fc="none",ec=C["spring"],lw=1.6,hatch="----",zorder=8))
        ax.add_patch(Polygon(P(rect(xr-5,xr+5,21.5,29.5)),fc=C["ear"],ec="k",lw=.6,alpha=.9,zorder=8))
    ax.add_patch(Polygon(P(rect(-11,11,0,9)),fc=C["col"],ec="none",zorder=3))
ax.add_patch(Circle((0,0),14.5,fc=C["col"],ec="k",lw=.8,zorder=3,alpha=.6))
ax.add_patch(Circle((0,0),7.5,fc="#6d7785",ec="k",lw=1,zorder=9)); ax.add_patch(Circle((0,0),4,fc="w",ec="k",lw=.8,zorder=9))
PI_A=286
ax.add_patch(Polygon(rect(-2.5,2.5,16,46)@R(PI_A).T,fc=C["pi"],ec="k",lw=.8,zorder=6,alpha=.85))
# dims
dim(ax,(-60,-72),(60,-72),"배관 내경 Ø120",va="top",off=(0,-1)); 
for x in (-60,60): ax.plot([x,x],[0,-74],color="k",lw=.4,ls=":")
dim(ax,(-50,-64.5),(50,-64.5),"몸체 외경 Ø100 (벽 2)",va="bottom"); 
for x in (-50,50): ax.plot([x,x],[0,-66],color="k",lw=.4,ls=":")
dim(ax,(-11.5,64),(11.5,64),"바퀴 폭 23"); 
lab(ax,(8,56),(30,70),"옴니휠 Ø38.1")
lab(ax,(17,26.95),(52,62),"모터 Ø25 (길이 방향으로 누움)\n바퀴 중심선에서 옆으로 17")
lab(ax,(17,42),(60,48),"웜휠 (모듈 0.8, 20T 예시)")
lab(ax,(-15,41),(-64,58),"허브 (3D 프린팅)\nM2×4, PCD 12",ha="right")
lab(ax,(-21.2,36),(-70,44),"축 지지판",ha="right")
lab(ax,(6,15),(-40,74),"가이드봉 Ø4 + 스프링 C202\n(바퀴당 2개)",ha="right")
lab(ax,(-6,25),(-66,70),"부싱 든 슬라이더",ha="right")
lab(ax,(3,-2),(-72,2),"가운데 기둥\n바깥 Ø15 / 안 Ø8 (배선)",ha="right")
lab(ax,(-12,-6),(-72,-14),"받침 블록 Ø29",ha="right")
p=R(PI_A)@np.array([0,36]); lab(ax,p,(70,8),"Pi Zero 2 W\n(세워서, 폭 30×두께 5)")
p0=R(240)@np.array([17,26.95]); lab(ax,p0,(52,-52),"모터끼리 간격 최소 25")
ax.text(0,-83,"점선 원 = 모터(바퀴 뒤쪽에 있어 단면에는 투영으로 표시)",ha="center",fontsize=9,color="#444")
ax.set_xlim(-98,98); ax.set_ylim(-88,82)
# ---------- side view ----------
ax=fig.add_axes([0.47,0.36,0.52,0.55]); ax.set_aspect("equal"); ax.axis("off")
ax.set_title("② 측면 단면 (위쪽 바퀴 유닛 1개만 표시, 왼쪽이 뒤 · 오른쪽이 앞)",fontsize=13,loc="left")
ax.add_patch(Rectangle((-112,60),152,5,fc="#eef1f4",ec=C["pipe"])); ax.add_patch(Rectangle((-112,-65),152,5,fc="#eef1f4",ec=C["pipe"]))
ax.plot([-112,40],[60,60],"k",lw=1.4); ax.plot([-112,40],[-60,-60],"k",lw=1.4)
for y0 in (48,-50): ax.add_patch(Rectangle((-88,y0),124,2,fc=C["body"],ec="k",lw=.8))
ax.add_patch(Rectangle((-88,-50),2,100,fc=C["body"],ec="k",lw=.8)); ax.add_patch(Rectangle((34,-50),2,100,fc=C["body"],ec="k",lw=.8))
ax.add_patch(Rectangle((-22,47.5),44,3,fc="w",ec="none",zorder=2))   # window
ax.add_patch(Rectangle((-86,-7.5),120,15,fc="#6d7785",ec="k",lw=.8)); ax.add_patch(Rectangle((-86,-4),120,8,fc="w",ec="k",lw=.5))
for z0 in (-30,20):
    ax.add_patch(Rectangle((z0,7.5),10,1.5,fc=C["col"],ec="k",lw=.6)); ax.add_patch(Rectangle((z0,-14.5),10,7,fc=C["col"],ec="k",lw=.6))
ax.add_patch(Polygon([[-30,21.5],[30,21.5],[30,29.5],[9,44.5],[-9,44.5],[-20,36],[-20,29.5],[-30,29.5]],fc=C["plate"],ec="k",lw=.6,alpha=.45,zorder=3))
ax.add_patch(Circle((0,40.95),19.05,fc=C["wheel"],ec="k",lw=1,alpha=.8,zorder=4))
ax.add_patch(Circle((0,40.95),8,fc="none",ec=C["gear"],lw=1.8,ls="--",zorder=6)); ax.add_patch(Circle((0,40.95),2,fc="k",zorder=7))
ax.add_patch(Rectangle((-5,20.15),10,13.6,fc=C["gear"],ec="k",lw=.7,hatch="///",zorder=6))
ax.add_patch(Rectangle((-11,24.95),20,4,fc="#444",ec="k",lw=.4,zorder=5))
ax.add_patch(Rectangle((-21,22.95),14,8,fc="#e8e0c8",ec="k",lw=.7,zorder=6))
ax.add_patch(Rectangle((-23,14),3,26,fc=C["plate"],ec="k",lw=.6,zorder=5))
ax.add_patch(Rectangle((-75,14.45),52,25,fc=C["motor"],ec="k",lw=1,zorder=5)); ax.add_patch(Rectangle((-84,14.75),9,24.4,fc="#5aa469",ec="k",lw=.8,zorder=5))
for z in (-25,25):
    ax.add_patch(Rectangle((z-2,3),4,31,fc=C["rod"],ec="k",lw=.5,zorder=7))
    zz=np.linspace(9,21.5,12); ax.plot(z+3.25*np.array([(-1)**i for i in range(12)]),zz,color=C["spring"],lw=1.8,zorder=8)
    ax.add_patch(Rectangle((z-5,21.5),10,8,fc=C["ear"],ec="k",lw=.6,alpha=.9,zorder=8))
    ax.add_patch(Rectangle((z-3.5,31.5),7,2.5,fc="#222",zorder=8))
ax.add_patch(Rectangle((36,-8),3,16,fc="#f0c040",ec="k",lw=.8))
ax.add_patch(Rectangle((-86,-40),65,5,fc=C["pi"],ec="k",lw=.7,ls="--",alpha=.55))
ax.text(-53.5,-37.5,"Pi Zero 2 W 65×30 (다른 칸에 있음, 길이 방향 위치만 표시)",ha="center",va="center",fontsize=8)
ax.plot([-112,44],[0,0],color="k",lw=.5,ls="-.")
dim(ax,(-88,-70),(36,-70),"몸체 길이 124",va="top",off=(0,-1)); 
for z in (-88,36): ax.plot([z,z],[-50,-72],color="k",lw=.4,ls=":")
dim(ax,(-84,-25),(-23,-25),"모터 61 (엔코더 내장)",va="top",off=(0,-1)); 
for z in (-84,-23): ax.plot([z,z],[14,-27],color="k",lw=.4,ls=":")
dim(ax,(-23,-17),(0,-17),"23",va="top",off=(0,-1)); dim(ax,(0,-17),(36,-17),"36",va="top",off=(0,-1)); ax.plot([0,0],[40,-19],color="k",lw=.4,ls=":")
dim(ax,(-25,70),(25,70),"가이드봉 간격 50"); 
for z in (-25,25): ax.plot([z,z],[34,72],color="k",lw=.4,ls=":")
dim(ax,(-116,0),(-116,40.95),"바퀴 축\n높이 40.95",ha="right",va="center",off=(-1.5,8)); ax.plot([-118,0],[40.95,40.95],color="k",lw=.4,ls=":")
dim(ax,(-98,0),(-98,26.95),"모터 축\n높이 26.95",ha="left",va="center",off=(1.5,-4)); ax.plot([-100,-84],[26.95,26.95],color="k",lw=.4,ls=":")
dim(ax,(-106,-60),(-106,0),"배관 내경\nØ120",ha="right",va="center",off=(-1.5,0))
lab(ax,(3,22),(46,-34),"웜 (모터 축과 같은 선)\n웜휠과 축간 거리 14")
lab(ax,(-14,26.95),(-60,52),"축 연장 슬리브\n(모터 축 Ø4 D컷 ↔ Ø4 봉)",ha="right")
lab(ax,(-21.5,36),(-36,66),"모터 고정판",ha="right")
lab(ax,(27,14),(46,12),"스프링 C202\n자유 16 → 설치 12.5")
lab(ax,(27,32.7),(46,40),"스토퍼 (바깥쪽 +2에서 멈춤)")
lab(ax,(38.5,-4),(46,-12),"ToF 센서\n(앞판 중앙)")
lab(ax,(-10,6),(-40,-12),"가운데 기둥 (속으로 배선)",ha="right")
ax.set_xlim(-150,112); ax.set_ylim(-78,78)
# ---------- notes ----------
ax=fig.add_axes([0.47,0.03,0.52,0.31]); ax.axis("off")
t1=("■ 주요 치수\n"
"· 배관 내경 120 / 몸체 외경 100 · 길이 124\n"
"· 바퀴가 몸체 밖으로 10 나와 벽에 닿음\n"
"· 모터 FIT0521: Ø25 × 61 (+축 12), 3개\n"
"· 가이드봉 Ø4 × 길이 31, 6개 (바퀴당 2개)\n"
"· 바퀴 축 Ø4 × 길이 약 43, 3개\n"
"· 웜기어 감속 1:10~1:20 → 약 20~40 mm/s\n"
"\n■ 스프링 (C202 를 바퀴당 2개)\n"
"· 설치 시 3.5 눌림 → 바퀴당 약 9.9 N\n"
"· 바깥쪽 +2 (엘보 내경 123.8) → 약 4.5 N\n"
"· 안쪽으로 3.4 더 들어감 (그 이상은 스프링 한계)")
t2=("■ 간섭 확인 (계산값, 스프링 이동 범위 전체)\n"
"· 모터 ↔ 몸체 안벽: 여유 1.9 이상\n"
"· 모터 ↔ 받침 블록: 여유 2.0 이상\n"
"· 모터 ↔ 모터: 여유 25 이상\n"
"· 모터 ↔ 옆 유닛 가이드봉: 여유 8.4 이상\n"
"· Pi Zero 2 W ↔ 주변: 여유 약 1.5~2.5 (빠듯함)\n"
"· 모터 고정판 ↔ 바퀴: 여유 약 1\n"
"\n■ 확인·구매가 필요한 것\n"
"· C202 안지름이 4 보다 큰지 (봉에 끼워야 함)\n"
"· 부싱 바깥지름·길이 (Ø7 × 8 로 가정)\n"
"· 웜·웜휠, 축 베어링(4×7) 6개, 허브·슬리브 출력\n"
"· 모터 드라이버·배터리는 뒤 모듈에 배치")
ax.text(0.0,1.0,t1,fontsize=10.5,va="top",linespacing=1.45); ax.text(0.47,1.0,t2,fontsize=10.5,va="top",linespacing=1.45)
fig.text(0.02,0.02,"배치 검토용 도면 · 2026-11-19 · 부품 치수는 제조사 도면 기준, 웜기어·부싱 치수는 가정값",fontsize=9,color="#555")
for f in sys.argv[1:]: fig.savefig(f)
