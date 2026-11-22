"""Export loose parts for the front-module dry assembly, in mm."""
from pathlib import Path
from math import cos, sin, radians
import cadquery as cq
import front_module as f

out = Path(__file__).with_name("fit_kit")
out.mkdir(exist_ok=True)
parts = {name: factory() for name, factory in f.PRINT.items()}
# Local coordinates put each dummy on its own printable base.
motor = f.fit0522()
for x in (-8.5, 8.5):
    motor = motor.cut(f.cylz(1.25, -6, 1, x))
parts["motor_gauge"] = motor
wheel = f.cylz(19.05, 0, 23).cut(f.cylz(2.1, -1, 24))
for i in range(4):
    angle = radians(45 + 90*i)
    wheel = wheel.cut(f.cylz(0.8, -1, 6, 6*cos(angle), 6*sin(angle)))
parts["wheel_gauge"] = wheel
board = f.box(-24.13, 24.13, -20.83, 20.83, 0, 1.6)
for x in (-21.59, 21.59):
    for y in (-18.29, 18.29):
        board = board.cut(f.cylz(1.5, -1, 2, x, y))
parts["driver_gauge"] = board.union(f.box(-17, 17, -14, 14, 1.6, 13.6))
parts["esp_gauge"] = f.box(0,54.3,0,27.94,0,1.6).union(f.box(0,54.3,2,26,1.6,4.8))
parts["worm_gauge"] = f.cylz(6.8,0,10).cut(f.cylz(2.1,-1,11))
# Toothless wheel has a flat facing the worm to clear the tooth engagement zone.
parts["gear_gauge"] = f.cylz(8.8,0,4).cut(f.cylz(2.1,-1,5)).cut(f.box(-10,10,-10,-7, -1,5))
parts["bush_gauge"] = f.cylz(3.5,0,8).cut(f.cylz(2.1,-1,9))
for name, length in (("wheel_bearing_gauge",2.5), ("worm_bearing_gauge",3)):
    parts[name] = f.cylz(3.5,0,length).cut(f.cylz(2.1,-1,length+1))
for name, part in parts.items():
    assert part.solids().size() == 1 and part.val().isValid(), name
    b = part.val().BoundingBox()
    cq.exporters.export(part.translate((-b.xmin,-b.ymin,-b.zmin)), str(out / (name+".stl")), tolerance=.02)
    print(f"{name}: {b.xlen:.1f} x {b.ylen:.1f} x {b.zlen:.1f} mm, {part.val().Volume()/1000:.1f} cm3")
