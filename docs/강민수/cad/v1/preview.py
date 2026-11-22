import sys, cadquery as cq, cairosvg
sys.argv=[sys.argv[0]]+sys.argv[1:]
import front_module as F
out=sys.argv[1]
P={n:f() for n,f in F.PRINT.items()}
for n,p in P.items(): print(n,"solids:",p.solids().size(),"valid:",p.val().isValid())
mv=dict(slider=P["slider"],hub=P["hub"],sleeve=P["sleeve"],**F.bought_moving()); fx=F.bought_fixed()
def comp(parts): return cq.Compound.makeCompound([p.val() for p in parts])
def svg(shape,name,d,w=1100,h=850):
    cq.exporters.export(cq.Workplane().add(shape),f"{out}/{name}.svg",opt=dict(width=w,height=h,marginLeft=20,marginTop=20,showAxes=False,projectionDir=d,strokeWidth=0.35,showHidden=False))
    cairosvg.svg2png(url=f"{out}/{name}.svg",write_to=f"{out}/{name}.png",background_color="white")
units=[q for k in range(3) for q in F.unit(k,0,mv).values()]
svg(comp([P["spine"]]+list(fx.values())+units),"조립_속구조",(1.2,0.9,1.6))
svg(comp([P["spine"],P["shell"],P["front_cap"].translate((0,0,25)),P["rear_cap"].translate((0,0,-25))]+list(fx.values())+units),"조립_전체",(1.2,0.9,1.6))
svg(comp(list(mv.values())),"유닛",(1.5,1.0,1.2),900,700)
svg(comp([P["slider"]]),"슬라이더",(1.5,1.0,1.2),900,700)
