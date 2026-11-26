"""앞 모듈 v2 3D 설명 페이지를 만든다. 실행: python 앞모듈_v2_3D_build.py  → 앞모듈_v2_3D_설명.html  (cadquery 필요)

- ../cad/v2/front_module_v2.py 의 실제 형상을 삼각형으로 나눠 HTML 안에 넣는다.
- three.js r128(MIT)을 cdnjs에서 받아 같이 넣는다. 그래서 만들어진 HTML은 인터넷 없이도 열린다.
  받지 못하면 CDN 주소를 그대로 쓰는 HTML을 만든다 (열 때 인터넷 필요).
- 화면 구성과 동작은 같은 폴더의 앞모듈_v2_3D_template.html 에 있다.
확인용 그림이며 치수 도면이 아니다. 스프링은 단순한 코일 모양으로 그린다.
"""
import sys, json, base64, pathlib, urllib.request
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "cad" / "v2"))
import front_module_v2 as F

THREE_URL = "https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"
SCALE = 50          # 좌표를 1/50 mm 정수로 저장

def mesh(wp, tol=0.12, ang=0.35):
    V, T, n = [], [], 0
    for s in wp.solids().vals():
        v, t = s.tessellate(tol, ang)
        V.append(np.array([(p.x, p.y, p.z) for p in v])); T.append(np.array(t) + n); n += len(v)
    V = np.concatenate(V); T = np.concatenate(T)
    q = np.round(V * SCALE).astype("<i2"); wide = len(V) > 65535
    idx = T.astype("<u4" if wide else "<u2")
    return dict(p=base64.b64encode(q.tobytes()).decode(), i=base64.b64encode(idx.tobytes()).decode(), w=int(wide), nv=len(V), nt=len(T))

def bearing(x0, x1, f0, f1):
    return F.cylx(3.5, x0, x1, F.A0, 0).union(F.cylx(4.1, f0, f1, F.A0, 0)).cut(F.cylx(2, x0 - 1, x1 + 1, F.A0, 0))

if __name__ == "__main__":
    P = {n: f() for n, f in F.PRINT.items()}; B = F.bought_moving()
    sp = P["spacer"].union(P["spacer"].translate((0, 0, F.GUIDE_Z[1] - F.GUIDE_Z[0])))
    shapes = dict(base=P["base"], front_cap=P["front_cap"], cover=P["cover"], rods=F.rods_unit(), spacer=sp,
                  slider=P["slider"], hub=P["hub"], wheel=B["wheel"], wormwheel=F.wormwheel(), worm=F.worm(), axle=B["axle"],
                  bearingB=bearing(13, 15.5, 12.4, 13), bearingC=bearing(28, 30.5, 30.5, 31.1), bushing=B["bushing"],
                  motor=B["motor"], plug=B["plug"])
    parts = {}
    for n, s in shapes.items():
        parts[n] = mesh(s); print(f"{n}: 꼭짓점 {parts[n]['nv']}, 삼각형 {parts[n]['nt']}", flush=True)
    consts = dict(scale=SCALE, A0=F.A0, WR=F.WR, MX=F.MX, MY=F.MY, seat=F.SEAT_Y, spacer=F.SPACER, earOff=F.EAR_OFF, guideZ=list(F.GUIDE_Z),
                  zFront=F.Z_FRONT, zRear=F.Z_REAR, dMin=F.D_MIN, dMax=F.D_MAX, dSolid=F.D_SOLID, coverFlat=F.TRI_D, coverR=F.TRI_RV,
                  length=F.Z_FRONT - F.Z_REAR, ratio=F.WHEEL_Z, springK=1.41, springFree=16.0, springMin=F.C202_LMIN)
    data = json.dumps(dict(parts=parts, c=consts), ensure_ascii=False, separators=(",", ":"))
    try:
        req = urllib.request.Request(THREE_URL, headers={"User-Agent": "Mozilla/5.0"})
        three = "<script>" + urllib.request.urlopen(req, timeout=30).read().decode("utf-8") + "</script>"; how = "three.js 포함 (오프라인 가능)"
    except Exception as e:
        three = f'<script src="{THREE_URL}"></script>'; how = f"three.js를 받지 못해 CDN 주소 사용 ({e})"
    html = (HERE / "앞모듈_v2_3D_template.html").read_text(encoding="utf-8")
    assert "/*__THREE__*/" in html and "/*__DATA__*/" in html
    html = html.replace("<script>/*__THREE__*/</script>", three).replace("/*__DATA__*/null", data)
    out = HERE / "앞모듈_v2_3D_설명.html"; out.write_text(html, encoding="utf-8")
    print(f"saved {out.name}: {out.stat().st_size / 1e6:.2f} MB, {how}")
