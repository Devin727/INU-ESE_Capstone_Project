"""Label the actual front_module CAD geometry for the design meeting."""
from pathlib import Path
import io
exec(Path(__file__).with_name('render_meeting.py').read_text(encoding='utf-8').split('\np={name:')[0])
from mpl_toolkits.mplot3d import proj3d

COLORS.update(slider='#4386BA', motor='#B9C1C8', hub='#E1B14C', sleeve='#DD9147',
              gear='#D7A63A', worm='#D7A63A', bearing='#C66643', axle='#596C7B', wormshaft='#596C7B')
moving=dict(slider=F.slider(),hub=F.hub(),sleeve=F.sleeve(),**F.bought_moving())
labels=[('motor','FIT0522 모터','배관 길이 방향으로 배치'),
        ('slider','슬라이더','바퀴·모터를 함께 지지'),
        ('wheel','옴니휠','CAD에서는 원통 외형으로 표현'),
        ('hub','허브','바퀴와 Ø4 mm 축 연결'),
        ('bearing','MF74ZZ 베어링 2개','양쪽 지지판에서 바퀴 축 지지'),
        ('gear','웜휠','바퀴 축에 달리는 기어'),
        ('worm','웜','웜휠과 직각으로 맞물림'),
        ('sleeve','슬리브','모터 출력축과 웜 축 연결')]

def draw(mode):
    offsets={} if mode=='assembled' else dict(motor=(0,0,-36),wheel=(-44,0,0),hub=(-23,0,0),bearing=(0,21,0),gear=(32,0,0),worm=(0,0,29),sleeve=(0,-19,-10),axle=(0,-15,0))
    parts={n:q.translate(offsets.get(n,(0,0,0))) for n,q in moving.items()}
    fig=plt.figure(figsize=(12,9),dpi=130,facecolor=BG)
    ax=fig.add_subplot(projection='3d')
    clouds=[]
    for n,q in parts.items():
        v,t=q.val().tessellate(.25,.22)
        pts=np.array([[v.z,v.x,v.y] for v in v]);clouds.append(pts)
        ax.add_collection3d(Poly3DCollection(pts[np.array(t)],facecolors=COLORS[n],edgecolors='#213B42',linewidth=.025,shade=True,lightsource=matplotlib.colors.LightSource(315,45)))
    pts=np.concatenate(clouds);lo=pts.min(0);hi=pts.max(0);span=hi-lo;mid=(hi+lo)/2
    for j,lim in enumerate((ax.set_xlim,ax.set_ylim,ax.set_zlim)):lim(mid[j]-.58*span[j],mid[j]+.58*span[j])
    ax.set_box_aspect(span);ax.view_init(27,-57);ax.set_axis_off();fig.subplots_adjust(0,0,1,1)
    fig.canvas.draw()
    buf=io.BytesIO();fig.savefig(buf,dpi=130,facecolor=BG);buf.seek(0)
    im=Image.new('RGB',(2200,1450),BG);im.paste(Image.open(buf).convert('RGB'),(0,170));d=ImageDraw.Draw(im)
    title='우리가 설계한 바퀴 유닛 · '+('조립 상태' if mode=='assembled' else '3D 분해도')
    text(d,65,35,title,52,True)
    text(d,68,111,'front_module.py의 실제 CAD 형상으로 렌더링  ·  '+('현재 부품 위치 그대로 표시' if mode=='assembled' else '부품 위치를 벌려 표시 · 배치 간격은 설명용'),27,color=MUTED)
    for i,(name,title,desc) in enumerate(labels):
        # Anchor each leader on an actual CAD bounding-box centre.
        b=parts[name].val().BoundingBox();x,y,z=proj3d.proj_transform((b.zmin+b.zmax)/2,(b.xmin+b.xmax)/2,(b.ymin+b.ymax)/2,ax.get_proj())
        px,py=ax.transData.transform((x,y));anchor=(int(px),int(1170-py+170))
        yy=205+i*118
        d.line((anchor,(1500,yy+22),(1570,yy+22)),fill=COLORS[name],width=3)
        d.ellipse((anchor[0]-6,anchor[1]-6,anchor[0]+6,anchor[1]+6),fill=COLORS[name],outline='white',width=2)
        text(d,1590,yy,title,32,True,color=COLORS[name]);text(d,1590,yy+48,desc,24)
    text(d,65,1300,'현재 남은 설계 문제  ·  출력축–웜 축 2.5 mm 겹침 / 웜 축 물림 길이 부족',28,True,color='#A33F2E')
    text(d,65,1352,'스프링·부싱은 이 CAD에 실체 형상이 없어 생략  ·  구매품은 CAD의 단순 외형으로 표시',25,color=MUTED)
    text(d,65,1401,'INU 캡스톤  |  앞마디 B안  |  2026.11.20  |  실물 조립·주행 검증 전',22,color=MUTED)
    im.save(OUT/('cad-unit-'+mode+'.png'));plt.close(fig)

draw('assembled')
draw('exploded')
print(OUT)
