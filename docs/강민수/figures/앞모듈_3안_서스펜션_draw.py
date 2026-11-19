import numpy as np, math, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle
from matplotlib import font_manager as fm
FN='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'; fm.fontManager.addfont(FN)
plt.rcParams["font.family"]=fm.FontProperties(fname=FN).get_name(); plt.rcParams["axes.unicode_minus"]=False
C=dict(pipe="#9aa7b4",body="#d9dee4",col="#8c96a3",wheel="#3f6fb5",motor="#b9c0c9",spring="#e07b1a",rod="#555",ear="#2f9e6e",gear="#c9a227",pi="#c2185b",plate="#7fb3e6",hub="#e8e0c8",drv="#d9534f",esp="#7b5fc4",tof="#f0c040")
def dim(ax,p,q,txt,off=(0,0),ha="center",va="bottom",fs=8.5):
    ax.annotate("",xy=p,xytext=q,arrowprops=dict(arrowstyle="<->",lw=0.8,color="k"))
    ax.text((p[0]+q[0])/2+off[0],(p[1]+q[1])/2+off[1],txt,ha=ha,va=va,fontsize=fs,bbox=dict(fc="w",ec="none",pad=0.6,alpha=.85))
def lab(ax,xy,xt,txt,fs=8.5,ha="left"):
    ax.annotate(txt,xy=xy,xytext=xt,fontsize=fs,ha=ha,va="center",arrowprops=dict(arrowstyle="-",lw=0.7,color="#333"))
def unit_side(ax,d=0,seat=True):
    """one suspension unit, side view (z horizontal, y radial), travel d"""
    if seat:
        for z0 in (-30,20): ax.add_patch(Rectangle((z0,7.5),10,1.5,fc=C["col"],ec="k",lw=.6))
    ax.add_patch(Polygon(np.array([[-30,21.5],[30,21.5],[30,29.5],[9,44.5],[-9,44.5],[-20,36],[-20,29.5],[-30,29.5]])+[0,d],fc=C["plate"],ec="k",lw=.6,alpha=.45,zorder=3))
    ax.add_patch(Circle((0,40.95+d),19.05,fc=C["wheel"],ec="k",lw=1,alpha=.8,zorder=4))
    ax.add_patch(Circle((0,40.95+d),8,fc="none",ec=C["gear"],lw=1.8,ls="--",zorder=6)); ax.add_patch(Circle((0,40.95+d),2,fc="k",zorder=7))
    ax.add_patch(Rectangle((-5,20.15+d),10,13.6,fc=C["gear"],ec="k",lw=.7,hatch="///",zorder=6))
    ax.add_patch(Rectangle((-11,24.95+d),20,4,fc="#444",ec="k",lw=.4,zorder=5))
    ax.add_patch(Rectangle((-21,22.95+d),14,8,fc=C["hub"],ec="k",lw=.7,zorder=6))
    ax.add_patch(Rectangle((-23,14+d),3,26,fc=C["plate"],ec="k",lw=.6,zorder=5))
    ax.add_patch(Rectangle((-75,14.45+d),52,25,fc=C["motor"],ec="k",lw=1,zorder=5)); ax.add_patch(Rectangle((-84,14.75+d),9,24.4,fc="#5aa469",ec="k",lw=.8,zorder=5))
    for z in (-25,25):
        ax.add_patch(Rectangle((z-2,3),4,31,fc=C["rod"],ec="k",lw=.5,zorder=7))
        zz=np.linspace(9,21.5+d,12); ax.plot(z+3.25*np.array([(-1)**i for i in range(12)]),zz,color=C["spring"],lw=1.8,zorder=8)
        ax.add_patch(Rectangle((z-5,21.5+d),10,8,fc=C["ear"],ec="k",lw=.6,alpha=.9,zorder=8))
        ax.add_patch(Rectangle((z-3.5,31.5),7,2.5,fc="#222",zorder=8))
def unit_front(ax,d=0):
    ax.add_patch(Rectangle((-11,3),22,6,fc=C["col"],ec="k",lw=.6))
    ax.add_patch(Rectangle((-11.5,21.9+d),23,38.1,fc=C["wheel"],ec="k",lw=1,alpha=.85,zorder=5))
    ax.add_patch(Rectangle((-19.5,31.95+d),8,18,fc=C["hub"],ec="k",lw=.7,zorder=5))
    ax.add_patch(Rectangle((-22.5,34+d),2.5,10.5,fc=C["plate"],ec="k",lw=.7,zorder=5)); ax.add_patch(Rectangle((12,20+d),2.5,24.5,fc=C["plate"],ec="k",lw=.7,zorder=5))
    ax.add_patch(Rectangle((15,32.95+d),4,16,fc=C["gear"],ec="k",lw=.7,zorder=5))
    ax.plot([-24,20.5],[40.95+d]*2,color="k",lw=.8,ls="-.",zorder=6)
    ax.add_patch(Circle((17,26.95+d),12.5,fc=C["motor"],ec="k",lw=1,ls="--",alpha=.7,zorder=4)); ax.add_patch(Circle((17,26.95+d),6.8,fc=C["gear"],ec="k",lw=.6,hatch="///",alpha=.8,zorder=4))
    for xr in (6,-6):
        ax.add_patch(Rectangle((xr-2,3),4,31,fc=C["rod"],ec="k",lw=.5,zorder=7))
        zz=np.linspace(9,21.5+d,12); ax.plot(xr+3.25*np.array([(-1)**i for i in range(12)]),zz,color=C["spring"],lw=1.8,zorder=8)
        ax.add_patch(Rectangle((xr-5,21.5+d),10,8,fc=C["ear"],ec="k",lw=.6,alpha=.9,zorder=8)); ax.add_patch(Rectangle((xr-3.5,31.5),7,2.5,fc="#222",zorder=8))
def body_side(ax,zr,zf,col=(None,None),pipe=True):
    if pipe:
        for y0 in (60,-65): ax.add_patch(Rectangle((zr-22,y0),zf-zr+34,5,fc="#eef1f4",ec=C["pipe"]))
        ax.plot([zr-22,zf+12],[60,60],"k",lw=1.4); ax.plot([zr-22,zf+12],[-60,-60],"k",lw=1.4)
    for y0 in (48,-50): ax.add_patch(Rectangle((zr,y0),zf-zr,2,fc=C["body"],ec="k",lw=.8))
    ax.add_patch(Rectangle((zr,-50),2,100,fc=C["body"],ec="k",lw=.8)); ax.add_patch(Rectangle((zf-2,-50),2,100,fc=C["body"],ec="k",lw=.8))
    ax.add_patch(Rectangle((-22,47.5),44,3,fc="w",ec="none",zorder=2))
    c0,c1=col; ax.add_patch(Rectangle((c0,-7.5),c1-c0,15,fc="#6d7785",ec="k",lw=.8)); ax.add_patch(Rectangle((c0,-4),c1-c0,8,fc="w",ec="k",lw=.5))
    for z0 in (-30,20): ax.add_patch(Rectangle((z0,-14.5),10,7,fc=C["col"],ec="k",lw=.6))
    ax.plot([zr-22,zf+14],[0,0],color="k",lw=.5,ls="-.")
# ================= Figure 1: versions =================
V=[dict(t="A안  드라이버 앞칸 + ESP32  (Pi는 뒤 마디)",zr=-88,zf=48,col=(-86,30),drv=(31,44.6),brd=("ESP32 (핀 없이, 54×28)",C["esp"],54.3),
        note="· 모터 2개는 앞 드라이버, 1개는 뒤 드라이버\n· 관절 통과 선: 전원 2 + 통신 2 + 모터 2 = 약 6가닥\n· 드라이버가 ToF 바로 뒤라 앞쪽이 무거워짐"),
   dict(t="B안  드라이버 뒤칸 + ESP32  (Pi는 뒤 마디)",zr=-102,zf=36,col=(-84,34),drv=(-99.6,-86),brd=("ESP32 (핀 없이, 54×28)",C["esp"],54.3),
        note="· 드라이버가 관절 쪽이라 배터리 전원선이 가장 짧음\n· 관절 통과 선: A안과 같음 (약 6가닥)\n· 가운데 기둥이 뒤판까지 닿지 않아 뒤판 보강 필요"),
   dict(t="C안  드라이버 앞칸 + Pi Zero 2 W  (ESP32 없음)",zr=-88,zf=48,col=(-86,30),drv=(31,44.6),brd=("Pi Zero 2 W (65×30)",C["pi"],65),
        note="· 제어를 앞 마디에 모음, 엔코더 3개를 Pi가 직접 읽음\n· Pi 주변 여유 1.5 mm, 열 빠질 곳 없음 → 비추천\n· 관절 통과 선: 전원 2 + 모터 2 = 약 4가닥")]
fig=plt.figure(figsize=(19,13),dpi=105)
fig.suptitle("앞 모듈 배치 3가지 (배관 내경 120, 드라이버 MDD3A 1개를 앞 마디에)  —  단위 mm",fontsize=17,fontweight="bold",y=0.98)
for i,v in enumerate(V):
    ax=fig.add_axes([0.01,0.655-0.305*i,0.47,0.275]); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(v["t"],fontsize=13.5,loc="left",fontweight="bold")
    body_side(ax,v["zr"],v["zf"],v["col"]); unit_side(ax)
    z0,z1=v["drv"]; ax.add_patch(Rectangle((z0,-20.83),z1-z0,41.66,fc=C["drv"],ec="k",lw=.9,alpha=.85,zorder=6))
    ax.text((z0+z1)/2,0,"MDD3A",ha="center",va="center",rotation=90,fontsize=8.5,color="w",fontweight="bold",zorder=7)
    name,colr,L=v["brd"]; ax.add_patch(Rectangle((-84,-42),L,5,fc=colr,ec="k",lw=.7,ls="--",alpha=.6,zorder=6)); ax.text(-84+L/2,-35,name+" · 모터 사이 칸",ha="center",va="bottom",fontsize=8.5)
    ax.add_patch(Rectangle((v["zf"],-8),3,16,fc=C["tof"],ec="k",lw=.8)); ax.text(v["zf"]+5,0,"ToF",fontsize=8.5,va="center")
    dim(ax,(v["zr"],-72),(v["zf"],-72),f"몸체 길이 {v['zf']-v['zr']}",va="top",off=(0,-1))
    for z in (v["zr"],v["zf"]): ax.plot([z,z],[-50,-74],color="k",lw=.4,ls=":")
    dim(ax,(z0,26),(z1,26),"13.6",fs=8)
    ax.text(v["zr"]-20,66,"뒤 (관절 쪽)",fontsize=9,color="#555",ha="left",va="bottom"); ax.text(v["zf"]+12,66,"앞",fontsize=9,color="#555",ha="right",va="bottom")
    ax.set_xlim(-126,66); ax.set_ylim(-80,74)
    fig.text(0.47,0.86-0.305*i,v["note"],fontsize=11,va="top",linespacing=1.6)
# driver bay cross-section
ax=fig.add_axes([0.78,0.58,0.20,0.34]); ax.set_aspect("equal"); ax.axis("off"); ax.set_title("드라이버 칸 단면",fontsize=12)
ax.add_patch(Circle((0,0),60,fc="w",ec="k",lw=1.2)); ax.add_patch(Circle((0,0),50,fc=C["body"],ec="k")); ax.add_patch(Circle((0,0),48,fc="w",ec="k",lw=.6))
ax.add_patch(Rectangle((-24.13,-20.83),48.26,41.66,fc=C["drv"],ec="k",alpha=.85))
for sx in (-1,1):
    for sy in (-1,1): ax.add_patch(Circle((sx*21.59,sy*18.29),1.5,fc="w",ec="k",lw=.6))
dim(ax,(-24.13,-27),(24.13,-27),"48.26",va="top",off=(0,-1)); dim(ax,(30,-20.83),(30,20.83),"41.66",ha="left",va="center",off=(1.5,0))
ax.text(0,0,"MDD3A\n(축에 수직으로 세움)",ha="center",va="center",fontsize=9,color="w",fontweight="bold")
ax.text(0,-70,"모서리까지 반지름 31.9 → 몸체 안벽 48 안에 들어감\n이 칸에서는 가운데 기둥이 끝나야 함",ha="center",va="top",fontsize=9)
ax.set_xlim(-66,66); ax.set_ylim(-88,64)
fig.text(0.77,0.50,"■ 공통\n· 바퀴 유닛 3개, 모터 3개, ToF 는 세 안 모두 같음\n· 드라이버 칸 때문에 몸체가 124 → 136~138 로 길어짐\n· 엘보 통과 한계(대략 140)에 가까움\n   → 실물 엘보로 확인 필요\n· ESP32 는 핀 헤더를 떼야 들어감\n   (헤더 포함 시 여유 0.3)",fontsize=11,va="top",linespacing=1.6)
fig.text(0.77,0.27,"■ 추천: B안\n· 전원선이 짧고 앞쪽이 가벼워 ToF 주변이 비어 있음\n· 앞판에 나중에 카메라·LED 를 넣을 자리가 남음",fontsize=11.5,va="top",linespacing=1.6,color="#0b5394")
fig.text(0.03,0.015,"배치 검토용 도면 · 2026-11-19 · 웜기어·부싱 치수는 가정값",fontsize=9,color="#555")
fig.savefig(sys.argv[1])
# ================= Figure 2: suspension unit =================
fig=plt.figure(figsize=(19,11.5),dpi=105)
fig.suptitle("서스펜션 바퀴 유닛 (1개 단독)  —  단위 mm",fontsize=17,fontweight="bold",y=0.975)
ax=fig.add_axes([0.005,0.06,0.32,0.86]); ax.set_aspect("equal"); ax.axis("off"); ax.set_title("① 앞에서 본 모습 (설치 상태)",fontsize=13,loc="left")
th=np.linspace(math.radians(60),math.radians(120),60); ax.fill_between(60*np.cos(th),60*np.sin(th),66*np.sin(th)+0*th+ (66-60)*0,color="#eef1f4"); ax.plot(60*np.cos(th),60*np.sin(th),"k",lw=1.4)
ax.add_patch(Circle((0,0),7.5,fc="#6d7785",ec="k")); ax.add_patch(Circle((0,0),4,fc="w",ec="k",lw=.6)); unit_front(ax)
dim(ax,(-11.5,66),(11.5,66),"바퀴 폭 23"); dim(ax,(-27,21.9),(-27,60),"Ø38.1",ha="right",va="center",off=(-1,14)); ax.plot([-29,-11.5],[21.9,21.9],"k",lw=.4,ls=":"); ax.plot([-29,-5],[60,60],"k",lw=.4,ls=":")
dim(ax,(-6,-12),(6,-12),"봉 간격 12 (좌우)",va="top",off=(0,-1)); dim(ax,(0,-22),(17,-22),"모터 옆으로 17",va="top",off=(0,-1)); ax.plot([17,17],[26.95,-24],"k",lw=.4,ls=":"); ax.plot([0,0],[0,-24],"k",lw=.4,ls=":")
dim(ax,(40,26.95),(40,40.95),"축간 14",ha="left",va="center",off=(1.5,0)); ax.plot([17,42],[26.95,26.95],"k",lw=.4,ls=":"); ax.plot([20,42],[40.95,40.95],"k",lw=.4,ls=":")
lab(ax,(-15.5,45),(-32,50),"허브",ha="right"); lab(ax,(17,46),(32,58),"웜휠"); lab(ax,(22,20),(34,10),"모터 Ø25\n(안쪽 빗금 = 웜)")
lab(ax,(-21.2,36),(-32,36),"축 지지판",ha="right"); lab(ax,(-9,25),(-32,25),"슬라이더\n(부싱)",ha="right"); lab(ax,(-9,15),(-32,13),"스프링 C202",ha="right"); lab(ax,(-8,5),(-32,2),"받침 블록",ha="right")
lab(ax,(4,-4),(26,-8),"가운데 기둥"); ax.text(0,72,"배관 벽 (내경 120)",ha="center",fontsize=9)
ax.set_xlim(-50,52); ax.set_ylim(-34,78)
ax=fig.add_axes([0.33,0.42,0.40,0.48]); ax.set_aspect("equal"); ax.axis("off"); ax.set_title("② 옆에서 본 모습 (설치 상태, 왼쪽이 뒤)",fontsize=13,loc="left")
ax.add_patch(Rectangle((-96,60),136,5,fc="#eef1f4",ec=C["pipe"])); ax.plot([-96,40],[60,60],"k",lw=1.4)
ax.add_patch(Rectangle((-96,-7.5),136,15,fc="#6d7785",ec="k",lw=.8)); ax.add_patch(Rectangle((-96,-4),136,8,fc="w",ec="k",lw=.5)); unit_side(ax)
dim(ax,(-25,-14),(25,-14),"가이드봉 간격 50 (앞뒤)",va="top",off=(0,-1)); 
for z in (-25,25): ax.plot([z,z],[3,-16],"k",lw=.4,ls=":")
dim(ax,(-84,46),(-23,46),"모터 61"); dim(ax,(-23,52),(0,52),"23"); ax.plot([-23,-23],[40,54],"k",lw=.4,ls=":"); ax.plot([-84,-84],[39,48],"k",lw=.4,ls=":")
dim(ax,(48,3),(48,34),"봉 길이 31\n(6은 블록에 박힘)",ha="left",va="center",off=(1.5,0)); ax.plot([27,50],[34,34],"k",lw=.4,ls=":"); ax.plot([27,50],[3,3],"k",lw=.4,ls=":")
dim(ax,(36,9),(36,21.5),"12.5",ha="left",va="center",off=(1,0),fs=8)
lab(ax,(0,22),(8,-26),"웜 (Ø4 봉에 고정)"); lab(ax,(-14,24),(-30,-18),"축 연장 슬리브",ha="right"); lab(ax,(-21.5,16),(-56,-26),"모터 고정판",ha="right")
lab(ax,(25,32.7),(52,46),"스토퍼"); lab(ax,(-80,27),(-104,40),"엔코더\n(내장)",ha="right"); lab(ax,(12,52),(30,72),"옴니휠 Ø38.1")
ax.set_xlim(-124,84); ax.set_ylim(-36,78)
st=[(2,"바깥쪽 끝 (+2)\n엘보 등 내경이 넓은 곳","스프링 14.5 · 바퀴당 약 4.5 N",62),(0,"설치 상태 (내경 120)","스프링 12.5 · 바퀴당 약 9.9 N",60),(-3.4,"가장 눌린 상태 (−3.4)","스프링 9.1 (한계) · 바퀴당 약 19.5 N",56.6)]
for i,(d,t,s,wall) in enumerate(st):
    ax=fig.add_axes([0.33+0.135*i,0.07,0.13,0.30]); ax.set_aspect("equal"); ax.axis("off"); ax.set_title(t,fontsize=10.5)
    ax.add_patch(Rectangle((-34,wall),68,4,fc="#eef1f4",ec=C["pipe"])); ax.plot([-34,34],[wall,wall],"k",lw=1.4)
    ax.add_patch(Rectangle((-34,-7.5),68,15,fc="#6d7785",ec="k",lw=.8)); unit_side(ax,d)
    ax.text(0,-12,s,ha="center",va="top",fontsize=9); ax.set_xlim(-36,36); ax.set_ylim(-22,68)
fig.text(0.33,0.395,"③ 스프링이 움직이는 범위 (모터는 일부만 보임)",fontsize=13)
fig.text(0.755,0.90,"■ 유닛 1개의 부품\n\n구매해 둔 것\n· 옴니휠 NX-14166 ×1\n· 모터 FIT0521 ×1\n· 가이드봉 Ø4 (31 로 절단) ×2\n· 부싱 안지름 4 ×2\n· 스프링 C202 ×2\n\n새로 필요한 것\n· 웜 + 웜휠 1세트 (모듈 0.8, 20T 가정)\n· 바퀴 축: Ø4 봉 약 43\n· 웜 축: Ø4 봉 약 21\n· 축 베어링 4×7 ×2~3\n\n3D 프린팅\n· 슬라이더 (축 지지판·모터 고정판 일체)\n· 허브 (M2×4, PCD 12)\n· 축 연장 슬리브\n· 받침 블록 (기둥에 고정, 3유닛 공용)\n\n■ 움직이는 것 / 고정된 것\n· 움직임: 슬라이더·바퀴·웜기어·모터\n· 고정: 기둥·받침 블록·가이드봉·스토퍼\n\n■ 확인 필요\n· C202 안지름 > 4 인지\n· 부싱 바깥지름·길이 (Ø7×8 가정)",fontsize=10.5,va="top",linespacing=1.5)
fig.text(0.02,0.02,"배치 검토용 도면 · 2026-11-19 · 웜기어·부싱 치수는 가정값",fontsize=9,color="#555")
fig.savefig(sys.argv[2])
