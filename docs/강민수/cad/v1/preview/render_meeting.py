"""Render meeting figures from the current CAD and a proposed passive joint."""
from pathlib import Path
import sys
import math
import io
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import front_module as F

OUT = Path(__file__).parent / 'meeting'
OUT.mkdir(exist_ok=True)
W,H = 1800,1120
BG='#F3F5F3'; INK='#18372F'; MUTED='#53675F'
COLORS=dict(slider='#9DB9AD',hub='#D3AB54',sleeve='#D3AB54',wheel='#447CAE',gear='#D3AB54',worm='#D3AB54',axle='#9CA8AA',wormshaft='#9CA8AA',motor='#C8CED0',bearing='#536675',spine='#466E5E',rods='#8E9E9B',mdd3a='#BE7153',esp32='#678A73')
def font(size,b=False):
    return ImageFont.truetype('C:/Windows/Fonts/malgun'+('bd' if b else '')+'.ttf',size)
def text(draw,x,y,s,size=27,b=False,color=INK):
    draw.text((x,y),s,font=font(size,b),fill=color)
def page(title,sub):
    im=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(im)
    text(d,65,40,title,49,True);text(d,68,112,sub,24,color=MUTED)
    d.line((65,170,1735,170),fill='#C5D3CA',width=2)
    text(d,65,1070,'INU 캡스톤  |  앞마디 기구 B안  |  2026.11.20  |  설계 검토용',21,color=MUTED)
    return im,d
def render(parts,size=(1040,830),view=(18,-58)):
    fig=plt.figure(figsize=(size[0]/120,size[1]/120),dpi=120,facecolor=BG)
    ax=fig.add_subplot(111,projection='3d',computed_zorder=True)
    allpoints=[]
    for name,part in parts.items():
        vertices,faces=part.val().tessellate(.30,.22)
        # Display pipe length horizontally, with radial geometry in the other axes.
        points=np.array([[v.z,v.x,v.y] for v in vertices]);allpoints.append(points)
        mesh=Poly3DCollection(points[np.array(faces)],facecolors=COLORS.get(name.split('_')[-1],'#839B90'),edgecolors='#31443D',linewidth=.045,shade=True,lightsource=matplotlib.colors.LightSource(azdeg=320,altdeg=50))
        ax.add_collection3d(mesh)
    pts=np.concatenate(allpoints);lo=pts.min(0);hi=pts.max(0);span=hi-lo;mid=(lo+hi)/2
    for j,setlim in enumerate((ax.set_xlim,ax.set_ylim,ax.set_zlim)):setlim(mid[j]-span[j]*.57,mid[j]+span[j]*.57)
    ax.set_box_aspect(span);ax.view_init(*view);ax.set_axis_off();fig.subplots_adjust(0,0,1,1)
    buf=io.BytesIO();fig.savefig(buf,dpi=120,facecolor=BG);plt.close(fig);buf.seek(0)
    return Image.open(buf).convert('RGB')
def item(d,y,color,title,lines):
    d.rounded_rectangle((1170,y+7,1194,y+31),5,fill=color)
    text(d,1210,y,title,30,True)
    for i,line in enumerate(lines):text(d,1210,y+49+34*i,line,24,color=MUTED)

p={name:factory() for name,factory in F.PRINT.items()}
mv=dict(slider=p['slider'],hub=p['hub'],sleeve=p['sleeve'],**F.bought_moving())
parts=dict(spine=p['spine'],**F.bought_fixed())
for k in range(3):parts.update({f'u{k}_{n}':q for n,q in F.unit(k,0,mv).items()})
im,d=page('앞마디 전체 구조','현재 CAD의 내부 배치  ·  내부를 보기 위해 외피와 덮개를 생략')
im.paste(render(parts,(1090,785)),(35,190));d=ImageDraw.Draw(im)
item(d,230,COLORS['wheel'],'옴니휠 3개',['배관 둘레에 120° 간격 배치','큰 바퀴가 앞뒤 추진을 담당'])
item(d,390,COLORS['slider'],'움직이는 바퀴 유닛',['슬라이더·모터·바퀴가 함께 이동','가이드봉과 부싱으로 방향 유지'])
item(d,550,COLORS['spine'],'고정 중심 기둥',['가이드봉과 전자부를 지지','스프링 C202는 CAD 그림에서 생략'])
item(d,710,COLORS['mdd3a'],'제어부 배치안',['앞 ESP32 + MDD3A 1개','뒤 드라이버로 남은 모터 구동 제안'])
text(d,80,964,'설계 기준 배관 내경 120 mm  /  외피 외경 100 mm  /  전체 길이 142 mm',28,True)
text(d,80,1010,'현재 남은 간섭  ·  엔코더–뒤 플랜지 약 1 mm  /  모터축–웜 축 2.5 mm',24,color='#9B4937')
im.save(OUT/'01-front-module.png')

im,d=page('바퀴 유닛 하나의 구성','현재 CAD 형상  ·  기어 이빨과 옴니휠 롤러는 단순 외형으로 표현')
im.paste(render(mv,(1090,785),view=(24,-62)),(35,180));d=ImageDraw.Draw(im)
item(d,225,COLORS['motor'],'모터 FIT0522',['엔코더가 모터 회전을 측정','출력축 → 슬리브 → 웜 축'])
item(d,385,COLORS['gear'],'웜과 웜휠',['직각인 두 축 사이에 동력 전달','추가 감속비 1:20은 검토안'])
item(d,545,COLORS['bearing'],'바퀴 축 베어링 2개',['양쪽 지지판에 끼워 축을 지지','플랜지형 MF74ZZ 가정'])
item(d,705,COLORS['hub'],'허브와 Ø4 mm 바퀴 축',['허브가 축 회전을 바퀴에 전달','웜휠과 축의 체결 방식 미정'])
text(d,80,961,'조립 흐름  ·  모터 체결 → 허브·바퀴 결합 → 양쪽 베어링과 바퀴 축 설치',27,True)
text(d,80,1008,'슬리브의 웜 축 물림 부족과 축 끝 겹침은 재설계 필요  ·  현재 실물 구동 미검증',24,color='#9B4937')
im.save(OUT/'02-wheel-unit.png')

im,d=page('수동 관절로 엘보를 통과하는 제안','관절·뒤마디·엘보는 설명용 개념 형상  ·  CAD 설계 및 통과 시험 전')
def arrow(a,b,c='#287C65',width=7):
    d.line((*a,*b),fill=c,width=width);ang=math.atan2(b[1]-a[1],b[0]-a[0]);d.polygon([b]+[(b[0]-22*math.cos(ang+t),b[1]-22*math.sin(ang+t)) for t in (-.5,.5)],fill=c)
# A curved pipe shown as a broad centreline, without implying a specified bend radius.
path=[(160,725),(480,725)]
path += [(480+360*math.sin(t),365+360*math.cos(t)) for t in np.linspace(0,math.pi/2,65)]
path += [(840,245)]
def pipe_band(radius, color):
    d.rectangle((160,725-radius,480,725+radius),fill=color)
    d.rectangle((840-radius,245,840+radius,365),fill=color)
    angles=np.linspace(0,math.pi/2,100)
    outline=[(480+(360+radius)*math.sin(t),365+(360+radius)*math.cos(t)) for t in angles]
    outline += [(480+(360-radius)*math.sin(t),365+(360-radius)*math.cos(t)) for t in reversed(angles)]
    d.polygon(outline,fill=color)
pipe_band(103,'#CBD7D2')
pipe_band(90,'#FFFFFF')
def body(cx,cy,ang,label):
    rad=math.radians(ang);u=np.array([math.cos(rad),math.sin(rad)]);v=np.array([-u[1],u[0]])
    corners=[tuple(np.array([cx,cy])+a*u+b*v) for a,b in [(-110,-53),(110,-53),(110,53),(-110,53)]]
    d.polygon(corners,fill='#A4BDAF',outline=INK,width=4)
    for off in (-64,64):
        pt=np.array([cx,cy])+off*u
        d.ellipse((pt[0]-18,pt[1]-18,pt[0]+18,pt[1]+18),fill='#447CAE',outline=INK,width=2)
    text(d,cx-58,cy-45,label,25,True)
body(410,725,0,'뒤마디');body(699,642,-35,'앞마디')
d.line((520,725,609,705),fill='#D3AB54',width=22);d.ellipse((546,699,582,735),fill='#E8C276',outline=INK,width=3)
arrow((244,866),(431,866));arrow((965,509),(965,318))
text(d,130,908,'뒤마디가 추진하며 앞마디를 따라옴',25)
d.line((565,710,1035,615),fill=MUTED,width=2);text(d,1060,588,'수동 관절 제안',32,True)
text(d,1060,643,'벽 접촉에 따라 앞·뒤마디가 꺾임',25)
text(d,1060,690,'서로 직각인 두 축으로 휘는 관절 검토',25)
text(d,1060,737,'회전 범위·복원력·케이블 경로는 미정',25)
text(d,1060,252,'옴니휠의 작은 롤러',30,True)
text(d,1060,307,'옆방향 이동과 비틀림 저항을 줄임',25)
text(d,1060,354,'몸체를 돌릴 능동 구동 방식은 별도 검토',24)
text(d,1060,440,'팀에서 확인할 것',30,True)
text(d,1060,493,'엘보 치수와 마디 길이·관절 각도',25)
text(d,80,985,'회의 제안  ·  무구동 관절 모형을 목표 엘보에 넣어 걸림·바퀴 접촉·배선 여유 확인',27,True)
im.save(OUT/'03-passive-joint.png')
print(OUT)
