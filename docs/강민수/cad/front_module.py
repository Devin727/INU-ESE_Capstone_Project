"""앞 모듈 (B안) CAD — 배관 내경 120, 단위 mm.
좌표: Z = 배관 길이 방향(앞이 +), 유닛 좌표 x = 접선, y = 반지름. 유닛 k 는 Z 축으로 120k 도 회전.
실행: python front_module.py <출력 폴더>   (cadquery 필요)
"""
import sys, os, math, itertools
import cadquery as cq

# ---------------- 치수 ----------------
R_PIPE=60.0; R_OUT=50.0; R_IN=48.0
WHEEL_R=19.05; WHEEL_W=23.0; AX_Y=R_PIPE-WHEEL_R          # 바퀴 축 높이 40.95
CD=14.0; MX=19.0; MY=AX_Y-CD                               # 웜 축간 거리(가정), 모터 축 위치
M_R=12.5; M_FACE=-23.0; M_LEN=61.0                         # FIT0521
ROD_R=2.0; BUSH_R=3.55                                     # 가이드봉 Ø4, 부싱 바깥 Ø7(가정)+여유
SEAT_Y=9.0; EAR_Y0=21.5; EAR_Y1=29.5                       # 스프링 받침면, 슬라이더 귀
Z_FRONT=34.0; FL_Z0=-87.5; FL_Z1=-85.0                     # 몸체 앞끝, 뒤 플랜지
Z_SHELL0=-90.0; Z_CAP0=-106.0
ESP_ANG=-74.0                                              # ESP32 칸 각도
TRAVEL=(2.0,0.0,-3.4)

def box(x0,x1,y0,y1,z0,z1): return cq.Workplane("XY").box(x1-x0,y1-y0,z1-z0,centered=False).translate((x0,y0,z0))
def cylz(r,z0,z1,x=0,y=0): return cq.Workplane("XY").circle(r).extrude(z1-z0).translate((x,y,z0))
def cylx(r,x0,x1,y,z): return cq.Workplane("YZ").circle(r).extrude(x1-x0).translate((x0,y,z))
def cyly(r,y0,y1,x,z): return cylz(r,0,y1-y0).rotate((0,0,0),(1,0,0),-90).translate((x,y0,z))
def rotz(s,deg): return s.rotate((0,0,0),(0,0,1),deg)
def tube(ro,ri,z0,z1): return cylz(ro,z0,z1).cut(cylz(ri,z0-1,z1+1))
def polar(r,deg): a=math.radians(deg); return (-r*math.sin(a), r*math.cos(a))   # 유닛 축(+y)에서 반시계 deg

# ---------------- 출력 부품 ----------------
def slider():
    s=box(-22.5,14.5,18.5,21.5,-20,20)                                   # 바닥
    s=s.union(box(-22.5,-20,18.5,44.5,-20,12))                           # 축 지지판 A
    s=s.union(box(12,14.5,18.5,44.5,8,20)).union(box(12,14.5,34.5,44.5,-8,20))   # 축 지지판 B (웜 위로 넘어감)
    s=s.union(box(12,MX+4.5,MY-4.5,MY+4.5,7.5,10.5))                       # 웜 축 앞 베어링 자리
    s=s.union(box(1,12,EAR_Y0,EAR_Y1,20,30)).union(box(-11,-1,EAR_Y0,EAR_Y1,-30,-20))   # 귀(부싱 자리)
    s=s.union(box(1,14.5,18.5,EAR_Y1,18,20)).union(box(-22.5,-1,18.5,EAR_Y1,-20,-18))   # 앞·뒤 벽
    s=s.union(cylz(12.6,M_FACE,-20,MX,MY)).union(box(-1,MX,18.5,EAR_Y1,M_FACE,-20))       # 모터 고정판
    s=s.cut(box(11.9,MX+8,15,34.3,-7.5,7.5))                                # 웜 자리
    s=s.cut(cylx(20,-12,12,AX_Y,0))                                      # 바퀴 자리
    s=s.cut(cylx(BUSH_R,-23,-19,AX_Y,0)).cut(cylx(BUSH_R,11,15,AX_Y,0))  # 축 베어링 4x7
    s=s.cut(cylz(BUSH_R,7,11,MX,MY))                                     # 웜 축 베어링 4x7
    s=s.cut(cyly(BUSH_R,EAR_Y0-1,EAR_Y1+1,6,25)).cut(cyly(BUSH_R,EAR_Y0-1,EAR_Y1+1,-6,-25))   # 부싱
    s=s.cut(cylz(4.5,M_FACE-1,-19,MX,MY))                                # 모터 축 구멍
    for dx in (-8.5,8.5): s=s.cut(cylz(1.65,M_FACE-1,-19,MX+dx,MY))      # 모터 M3 x2, 간격 17 (가정)
    return s
def hub():
    h=cylx(9,-14.5,-11.5,AX_Y,0).union(cylx(5,-19.5,-14.5,AX_Y,0)).cut(cylx(2.05,-20,-11,AX_Y,0))
    for i in range(4):
        a=math.radians(45+90*i); h=h.cut(cylx(1.1,-15,-11,AX_Y+6*math.cos(a),6*math.sin(a)))
    return h.cut(cyly(1.3,AX_Y,AX_Y+6,-17,0))                            # M3 고정 나사
def sleeve():
    s=cylz(4,-21,-7,MX,MY).cut(cylz(2.05,-22,-6,MX,MY))
    for z in (-17.5,-10): s=s.cut(cyly(1.3,MY,MY+5,MX,z))
    return s
def spine():
    s=cylz(7.5,FL_Z0,Z_FRONT).union(cylz(47.6,FL_Z0,FL_Z1))
    for z0,z1,xr in ((20,30,6),(-30,-20,-6)):
        b=cylz(14.5,z0,z1)
        for k in range(3): b=b.cut(rotz(box(-20,20,SEAT_Y,20,z0-1,z1+1),120*k))
        s=s.union(b)
        for k in range(3): s=s.cut(rotz(cyly(1.95,3,SEAT_Y+1,xr,(z0+z1)/2),120*k))       # 가이드봉 압입 구멍
    for sx,sy in itertools.product((-21.59,21.59),(-18.29,18.29)):                      # MDD3A 받침 (뒤쪽)
        s=s.union(cylz(3,Z_SHELL0,FL_Z0,sx,sy)).cut(cylz(1.2,Z_SHELL0-1,FL_Z1+1,sx,sy))
    s=s.cut(cylz(4,FL_Z0-5,Z_FRONT+1))                                                   # 배선 구멍
    for k in range(3):
        x,y=polar(44,60+120*k); s=s.cut(cylz(1.3,FL_Z0-1,FL_Z1+1,x,y))                   # M3 (몸체·뒤 덮개 체결)
        x,y=polar(36,120*k); s=s.cut(cylz(4,FL_Z0-1,FL_Z1+1,x,y))                        # 모터 선 구멍
    u=box(-2.6,2.6,14,17.6,FL_Z1,-70).union(box(-2.6,2.6,42.4,46,FL_Z1,-70)).cut(box(-1,1,15.5,44.5,FL_Z1-0.01,-69))
    return s.union(rotz(u,ESP_ANG))                                                      # ESP32 끼우는 홈
def shell():
    s=tube(R_OUT,R_IN,Z_SHELL0,Z_FRONT).union(tube(R_IN,42,Z_SHELL0,FL_Z0))
    for k in range(3):
        s=s.cut(rotz(box(-24,24.5,40,56,-22,Z_FRONT+1),120*k))                           # 바퀴 창 (앞으로 열림)
        x,y=polar(44,60+120*k); s=s.cut(cylz(1.7,Z_SHELL0-1,FL_Z0+1,x,y))
        s=s.cut(rotz(cyly(1.7,44,52,0,31.5),60+120*k))                                   # 앞 덮개 고정 M3
    return s
def front_cap():
    s=cylz(R_OUT,Z_FRONT,Z_FRONT+2).union(tube(47.7,44.5,30.5,Z_FRONT)).union(tube(10,7.7,31,Z_FRONT))
    s=s.cut(cylz(4,Z_FRONT-1,Z_FRONT+3))
    for k in range(3): s=s.cut(rotz(cyly(1.3,43,49,0,31.5),60+120*k))
    return s
def rear_cap():
    s=tube(R_OUT,R_IN,Z_CAP0,Z_SHELL0).union(cylz(R_OUT,Z_CAP0,Z_CAP0+2)).cut(cylz(7,Z_CAP0-1,Z_CAP0+3))
    for k in range(3):
        x,y=polar(44,60+120*k); s=s.union(cylz(4,Z_CAP0,Z_SHELL0,x,y).intersect(cylz(R_OUT,Z_CAP0,Z_SHELL0)))
    for k in range(3):
        x,y=polar(44,60+120*k); s=s.cut(cylz(1.7,Z_CAP0-1,Z_SHELL0+1,x,y))
    return s

# ---------------- 구매품 (간섭 확인·조립도용) ----------------
def bought_moving():
    w=cylx(WHEEL_R,-WHEEL_W/2,WHEEL_W/2,AX_Y,0)
    g=cylx(8.8,MX-2,MX+2,AX_Y,0); wm=cylz(6.8,-5,5,MX,MY)
    ax=cylx(2,-23.5,MX+2.5,AX_Y,0); ws=cylz(2,-11,10.5,MX,MY)
    m=cylz(M_R,M_FACE-M_LEN,M_FACE,MX,MY).union(cylz(2,M_FACE,M_FACE+12,MX,MY))
    return dict(wheel=w,gear=g,worm=wm,axle=ax,wormshaft=ws,motor=m)
def bought_fixed():
    d={}
    rods=None; springs=None
    for k in range(3):
        for xr,z in ((6,25),(-6,-25)):
            r=rotz(cyly(ROD_R,3,34,xr,z).union(cyly(3.5,31.5,34,xr,z)),120*k); rods=r if rods is None else rods.union(r)
    d["rods"]=rods
    d["mdd3a"]=box(-24.13,24.13,-20.83,20.83,Z_SHELL0-13.6,Z_SHELL0)
    d["esp32"]=rotz(box(-0.8,0.8,16,43.94,-84.5,-30.2),ESP_ANG)
    return d
PRINT=dict(slider=slider,hub=hub,sleeve=sleeve,spine=spine,shell=shell,front_cap=front_cap,rear_cap=rear_cap)

def vol(a,b):
    try: return a.intersect(b).val().Volume()
    except Exception: return 0.0
def unit(k,d,parts):
    return {n:rotz(p.translate((0,d,0)),120*k) for n,p in parts.items()}

if __name__=="__main__":
    out=sys.argv[1]; os.makedirs(out,exist_ok=True)
    P={n:f() for n,f in PRINT.items()}
    mv=dict(slider=P["slider"],hub=P["hub"],sleeve=P["sleeve"],**bought_moving())
    fx=dict(spine=P["spine"],shell=P["shell"],front_cap=P["front_cap"],rear_cap=P["rear_cap"],**bought_fixed())
    bad=[]
    # 유닛 내부 (부품끼리)
    for a,b in itertools.combinations(mv,2):
        v=vol(mv[a],mv[b])
        if v>0.5 and {a,b} not in ({"axle","wheel"},{"axle","gear"},{"axle","hub"},{"worm","wormshaft"},{"sleeve","wormshaft"},{"sleeve","motor"},{"gear","worm"}): bad.append((f"unit:{a}",b,round(v,1)))
    for d in TRAVEL:
        u0=unit(0,d,mv)
        for n,p in u0.items():
            for fn,fp in fx.items():
                v=vol(p,fp)
                if v>0.5: bad.append((f"d={d} {n}",fn,round(v,1)))
        for d2 in TRAVEL:
            u1=unit(1,d2,mv)
            for n,p in u0.items():
                for n2,p2 in u1.items():
                    v=vol(p,p2)
                    if v>0.5: bad.append((f"d={d}/{d2} u0:{n}","u1:"+n2,round(v,1)))
    for a,b in itertools.combinations(fx,2):
        v=vol(fx[a],fx[b])
        if v>0.5 and {a,b}!={"rods","spine"}: bad.append((a,b,round(v,1)))
    print("INTERFERENCE:",bad if bad else "none")
    for n,p in P.items():
        cq.exporters.export(p,f"{out}/{n}.stl",tolerance=0.02,angularTolerance=0.15); cq.exporters.export(p,f"{out}/{n}.step")
        bb=p.val().BoundingBox(); print(f"{n}: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm, {p.val().Volume()/1000:.1f} cm3")
    asm=cq.Assembly(name="front_module")
    col=dict(slider="lightskyblue",hub="khaki",sleeve="khaki",wheel="royalblue",gear="goldenrod",worm="goldenrod",axle="gray40",wormshaft="gray40",motor="gray70",
             spine="slategray",shell="gray85",front_cap="gray85",rear_cap="gray85",rods="gray30",mdd3a="indianred",esp32="mediumpurple")
    for n,p in fx.items(): asm.add(p,name=n,color=cq.Color(col[n]))
    for k in range(3):
        for n,p in unit(k,0,mv).items(): asm.add(p,name=f"u{k}_{n}",color=cq.Color(col[n]))
    asm.save(f"{out}/front_module_assembly.step")
