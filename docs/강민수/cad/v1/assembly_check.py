"""Read-only sampled assembly checks; run with Python + CadQuery.

No STL/STEP regeneration. Purchased parts use front_module.py envelopes.
No screws, wires, springs, bearing/bushing bodies or tool envelopes included.
Discrete samples are not a continuous collision-free path proof.
"""
import json
import sys
import cadquery as cq
import front_module as F


def overlap(a, b):
    aa, bb = a.val().BoundingBox(), b.val().BoundingBox()
    if any(getattr(aa, axis + 'max') <= getattr(bb, axis + 'min') or
           getattr(bb, axis + 'max') <= getattr(aa, axis + 'min')
           for axis in ('x', 'y', 'z')):
        return 0.0
    # Deliberately propagate Boolean errors rather than report them as clear.
    return sum(s.Volume() for s in a.intersect(b).solids().vals())


def check(label, moving, fixed, shifts):
    hits = []
    for shift in shifts:
        for name, part in moving.items():
            shifted = part.translate((0, 0, shift))
            for target, obstacle in fixed.items():
                volume = overlap(shifted, obstacle)
                if volume > 0.001:
                    hits.append(dict(z_shift_mm=shift, part=name, obstacle=target,
                                     overlap_mm3=round(volume, 6)))
    result = dict(check=label, samples=len(shifts), hits=hits)
    print(json.dumps(result), flush=True)
    return result


def main():
    print('Python:', sys.version.split()[0], 'CadQuery:', cq.__version__, flush=True)
    p = {name: factory() for name, factory in F.PRINT.items()}
    for name, part in p.items():
        bb = part.val().BoundingBox()
        print(name, 'solids:', part.solids().size(), 'valid:', part.val().isValid(),
              'size_mm:', tuple(round(x, 3) for x in (bb.xlen, bb.ylen, bb.zlen)), flush=True)
    moving = dict(slider=p['slider'], hub=p['hub'], sleeve=p['sleeve'], **F.bought_moving())
    fixed = dict(spine=p['spine'], **F.bought_fixed())
    for travel in F.TRAVEL:
        internals = dict(fixed)
        for k in range(3):
            internals.update({f'u{k}_{name}': part for name, part in F.unit(k, travel, moving).items()})
        check(f'front insertion; all units radial {travel} mm', internals,
              {'shell': p['shell']}, list(range(140, -1, -2)))
        check(f'front cap insertion; all units radial {travel} mm',
              {'front_cap': p['front_cap']}, dict(shell=p['shell'], **internals),
              list(range(40, -1, -2)))
        check(f'rear cap insertion; all units radial {travel} mm',
              {'rear_cap': p['rear_cap']}, dict(shell=p['shell'], **internals),
              list(range(-40, 1, 2)))
        # Include all three units against asymmetric ESP32 and other fixed parts.
        for k in range(3):
            check(f'final unit{k}; radial {travel} mm', F.unit(k, travel, moving),
                  dict(shell=p['shell'], front_cap=p['front_cap'], rear_cap=p['rear_cap'], **fixed), [0])
    check('motor inserted axially from rear, isolated slider',
          {'motor': moving['motor']}, {'slider': p['slider']}, list(range(-80, 1, 2)))


if __name__ == '__main__':
    main()
