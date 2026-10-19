"""부품 배치 검사: models/<model>/layout.yaml 의 부품이 마디 안에 들어가는지.

ros2 run pipe_robot_sim layout_check two_real [--apply]

검사 항목 (external 부품은 1번 제외)
  1. 몸통 안쪽(외경 - 2 x 벽) 과 마디 길이 안에 있는가
  2. 바퀴 모듈 공간(바퀴 고리 ±x_half, 각 바퀴 방향 반폭, 반경 r_min 바깥) 을 침범하지 않는가
  3. 다른 부품과 겹치지 않는가
출력: 문제 목록, 마디별 질량·무게중심, models/<model>/components.urdf.xacro (Gazebo 에 보이는 부품 상자)
--apply: robot.yaml 의 seg_len, seg_mass, seg_com, spawn_x 갱신 (소스 폴더)
"""
import argparse
import math
import os
import re

import numpy as np
import yaml

SRC_MODELS = os.path.expanduser('~/Capstone_sim/src/pipe_robot_sim/models')


def samples(part, n=7):
    """부품 부피 안 점들 (mm, 마디 좌표)."""
    shape, pos = part['shape'], np.array(part['pos'], float)
    if shape == 'box':
        sx, sy, sz = part['size']
        g = [np.linspace(-s / 2, s / 2, n) for s in (sx, sy, sz)]
        pts = np.array(np.meshgrid(*g)).reshape(3, -1).T
    elif shape in ('cyl_x', 'cyl_z'):
        d, ln = part['size']
        rr = np.linspace(0, d / 2, n)
        aa = np.linspace(0, 2 * math.pi, 4 * n, endpoint=False)
        ll = np.linspace(-ln / 2, ln / 2, n)
        disk = np.array([(r * math.cos(a), r * math.sin(a)) for r in rr for a in aa])
        pts = np.array([(l, u, v) for l in ll for u, v in disk])
        if shape == 'cyl_z':
            pts = pts[:, [1, 2, 0]]
    else:
        return np.zeros((0, 3))
    return pts + pos


def inside(part, pts, shrink=0.5):
    """pts 가 부품 부피 안인지 (경계는 shrink 만큼 봐줌)."""
    q = pts - np.array(part['pos'], float)
    if part['shape'] == 'box':
        h = np.array(part['size'], float) / 2 - shrink
        return np.all(np.abs(q) <= h, axis=1)
    d, ln = part['size']
    if part['shape'] == 'cyl_x':
        return (np.abs(q[:, 0]) <= ln / 2 - shrink) & (np.hypot(q[:, 1], q[:, 2]) <= d / 2 - shrink)
    if part['shape'] == 'cyl_z':
        return (np.abs(q[:, 2]) <= ln / 2 - shrink) & (np.hypot(q[:, 0], q[:, 1]) <= d / 2 - shrink)
    return np.zeros(len(q), bool)


def in_wheel_zone(pts, wm):
    r = np.hypot(pts[:, 1], pts[:, 2])
    ang = np.degrees(np.arctan2(pts[:, 2], pts[:, 1])) % 360
    hit = np.zeros(len(pts), bool)
    for a in wm['angles']:
        da = np.radians((ang - a + 180) % 360 - 180)
        tang = np.abs(r * np.sin(da))
        hit |= (np.abs(pts[:, 0]) <= wm['x_half']) & (r >= wm['r_min']) & (tang <= wm['half_width']) & (np.cos(da) > 0)
    return hit


def check(layout):
    hull, wm = layout['hull'], layout['wheel_module']
    r_in = hull['outer_d'] / 2 - hull['wall']
    report = {}
    for seg_name, seg in layout['segments'].items():
        L = seg['length']
        parts = seg['parts']
        probs = {p['name']: [] for p in parts}
        pts = {p['name']: samples(p) for p in parts}
        for p in parts:
            P = pts[p['name']]
            if len(P) == 0:
                continue
            if not p.get('external'):
                rr = np.hypot(P[:, 1], P[:, 2])
                if rr.max() > r_in + 0.3:
                    probs[p['name']].append(f'몸통 밖으로 {rr.max() - r_in:.1f} mm')
                if np.abs(P[:, 0]).max() > L / 2 + 0.3:
                    probs[p['name']].append(f'마디 길이 밖으로 {np.abs(P[:, 0]).max() - L / 2:.1f} mm')
            if in_wheel_zone(P, wm).any():
                probs[p['name']].append('바퀴 모듈 공간 침범')
        for i, a in enumerate(parts):
            for b in parts[i + 1:]:
                if b['name'] in a.get('allow', []) or a['name'] in b.get('allow', []):
                    continue
                if len(pts[a['name']]) and a['shape'] != 'mass' and b['shape'] != 'mass':
                    if inside(b, pts[a['name']]).any() or inside(a, pts[b['name']]).any():
                        probs[a['name']].append(f"{b['name']} 와 겹침")
                        probs[b['name']].append(f"{a['name']} 와 겹침")
        # 질량·무게중심 (몸통 껍데기 포함)
        shell = math.pi * ((hull['outer_d'] / 2) ** 2 - r_in ** 2) * L * hull['density'] / 1000.0
        caps = 2 * math.pi * r_in ** 2 * hull['wall'] * hull['density'] / 1000.0
        m = shell + caps + sum(p['mass'] for p in parts)
        com = sum(np.array(p['pos'], float) * p['mass'] for p in parts) / m
        report[seg_name] = dict(length=L, mass=m, com=com, hull=shell + caps, probs=probs,
                                est=[p['name'] for p in parts if p.get('est')])
    return report


COLORS = {'cell': '0.9 0.6 0.1', 'esp32': '0.1 0.5 0.9', 'pi': '0.1 0.7 0.3', 'servo': '0.15 0.15 0.15',
          'probe': '0.8 0.8 0.85', 'pump': '0.6 0.2 0.8', 'tank': '0.6 0.8 1.0', 'stepper': '0.4 0.4 0.4'}


def write_xacro(layout, report, robot_params, path):
    gap = robot_params['joint_gap']
    links = {'front': ('base_link', 0.0)}
    L1 = layout['segments']['rear']['length'] / 1000 if 'rear' in layout['segments'] else 0
    links['rear'] = ('mid_link', -(gap / 2 + L1 / 2))
    out = ['<?xml version="1.0"?>', '<!-- layout_check 로 생성: layout.yaml 의 부품 (보이기만, 충돌·질량 없음). 빨강 = 배치 문제 -->',
           '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">']
    for seg_name, seg in layout['segments'].items():
        link, off = links[seg_name]
        for p in seg['parts']:
            if p['shape'] == 'mass':
                continue
            x, y, z = (np.array(p['pos'], float) / 1000).tolist()
            x += off
            bad = bool(report[seg_name]['probs'][p['name']])
            col = '1 0.1 0.1' if bad else next((c for k, c in COLORS.items() if k in p['name']), '0.3 0.8 0.6')
            if p['shape'] == 'box':
                geo = '<box size="{} {} {}"/>'.format(*[s / 1000 for s in p['size']])
                rpy = '0 0 0'
            else:
                d, ln = p['size']
                geo = f'<cylinder radius="{d / 2000}" length="{ln / 1000}"/>'
                rpy = f'0 {math.pi / 2} 0' if p['shape'] == 'cyl_x' else '0 0 0'
            out.append(f'  <link name="part_{p["name"]}_vis"><visual><origin xyz="{x} {y} {z}" rpy="{rpy}"/>'
                       f'<geometry>{geo}</geometry><material name="part_{p["name"]}_mat"><color rgba="{col} 1"/></material>'
                       f'</visual></link>')
            out.append(f'  <joint name="part_{p["name"]}_fix" type="fixed"><parent link="{link}"/>'
                       f'<child link="part_{p["name"]}_vis"/></joint>')
    out.append('</robot>')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out) + '\n')


def apply_params(path, report, robot_params):
    s = open(path, encoding='utf-8').read()
    f, r = report['front'], report['rear']
    gap = robot_params['joint_gap']
    seg_len = f'[{f["length"] / 1000:.3f}, {r["length"] / 1000:.3f}, 0.0]'
    seg_mass = f'[{f["mass"] / 1000:.3f}, {r["mass"] / 1000:.3f}, 0.0]'
    # ROS 2 파라미터는 중첩 리스트 불가 -> [앞 x,y,z, 뒤 x,y,z, 0,0,0] 한 줄
    seg_com = '[' + ', '.join(f'{v / 1000:.4f}' for seg in (f, r) for v in seg['com']) + ', 0.0, 0.0, 0.0]'
    spawn = f['length'] / 1000 + gap + r['length'] / 1000 - f['length'] / 2000 + 0.02
    for k, v in [('seg_len', seg_len), ('seg_mass', seg_mass), ('seg_com', seg_com), ('spawn_x', f'{spawn:.3f}'),
                 ('body_len', f'{f["length"] / 1000:.3f}')]:
        if re.search(rf'^    {k}:', s, re.M):
            s = re.sub(rf'^(    {k}:) [^#\n]*', rf'\1 {v} ', s, flags=re.M)
        else:
            s = s.replace('  ros__parameters:\n', f'  ros__parameters:\n    {k}: {v}\n', 1)
    open(path, 'w', encoding='utf-8').write(s)


def plot(layout, report, robot_params, path):
    """배치도: 옆(x-z), 위(x-y), 바퀴 고리 단면(y-z). 빨강 = 문제 부품"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import patches
    plt.rcParams['font.family'] = ['NanumGothic', 'Noto Sans CJK JP', 'Noto Serif CJK JP', 'DejaVu Sans']
    hull, wm = layout['hull'], layout['wheel_module']
    R, r_in = hull['outer_d'] / 2, hull['outer_d'] / 2 - hull['wall']
    gap = robot_params['joint_gap'] * 1000
    segs = list(layout['segments'].items())
    offs, x = [], 0.0
    for i, (_, seg) in enumerate(segs):           # 앞마디 중심 = 0, 뒤로 갈수록 -
        if i == 0:
            offs.append(0.0)
            x = -seg['length'] / 2
        else:
            offs.append(x - gap - seg['length'] / 2)
            x = offs[-1] - seg['length'] / 2
    fig = plt.figure(figsize=(15, 9))
    axs = [fig.add_subplot(2, 1, 1), fig.add_subplot(2, 3, 4), fig.add_subplot(2, 3, 5), fig.add_subplot(2, 3, 6)]
    side, top = axs[0], None
    titles = ['옆에서 본 단면 (x-z, 오른쪽이 앞)']
    for (name, seg), off in zip(segs, offs):
        L = seg['length']
        side.add_patch(patches.Rectangle((off - L / 2, -R), L, 2 * R, fill=False, lw=1.5))
        side.add_patch(patches.Rectangle((off - wm['x_half'], -R), 2 * wm['x_half'], 2 * R, color='0.85', zorder=0))
        side.text(off, R + 3, f'{name} {L} mm, {report[name]["mass"]:.0f} g', ha='center')
        for prt in seg['parts']:
            if prt['shape'] == 'mass':
                continue
            bad = bool(report[name]['probs'][prt['name']])
            col = 'red' if bad else ('orange' if 'cell' in prt['name'] else 'tab:blue')
            px, py, pz = prt['pos']
            if prt['shape'] == 'box':
                sx, sy, sz = prt['size']
            elif prt['shape'] == 'cyl_x':
                sx, sy, sz = prt['size'][1], prt['size'][0], prt['size'][0]
            else:
                sx, sy, sz = prt['size'][0], prt['size'][0], prt['size'][1]
            side.add_patch(patches.Rectangle((off + px - sx / 2, pz - sz / 2), sx, sz, alpha=0.45, color=col))
            side.text(off + px, pz, prt['name'], fontsize=6, ha='center', va='center')
    side.axhline(0, color='0.7', lw=0.5)
    side.set_aspect('equal')
    side.set_xlim(offs[-1] - segs[-1][1]['length'] / 2 - 15, segs[0][1]['length'] / 2 + 20)
    side.set_ylim(-R - 40, R + 12)
    side.set_title(titles[0] + ' — 회색 띠 = 바퀴 고리(부품 금지)')
    # 단면: 앞마디 바퀴 고리 / 뒷마디 바퀴 고리 / 뒷마디 앞쪽 구역
    cuts = [(segs[0][0], 0.0, '앞마디 바퀴 고리 단면 (x=0)'), (segs[-1][0], 0.0, '뒷마디 바퀴 고리 단면 (x=0)'),
            (segs[-1][0], 36.0, '뒷마디 앞쪽 단면 (x=+36)')]
    for ax, (name, xc, ttl) in zip(axs[1:], cuts):
        ax.add_patch(patches.Circle((0, 0), R, fill=False, lw=1.5))
        ax.add_patch(patches.Circle((0, 0), r_in, fill=False, lw=0.5, ls='--'))
        ax.add_patch(patches.Circle((0, 0), 50.4, fill=False, lw=0.8, color='tab:green'))
        if abs(xc) <= wm['x_half']:
            for a in wm['angles']:
                t = np.radians(a)
                u, v = np.array([np.cos(t), np.sin(t)]), np.array([-np.sin(t), np.cos(t)])
                c0 = u * wm['r_min']
                ax.add_patch(patches.Polygon([c0 - v * wm['half_width'], c0 + v * wm['half_width'],
                                              u * 50.4 + v * wm['half_width'], u * 50.4 - v * wm['half_width']],
                                             color='0.8'))
        seg = dict(segs)[name]
        for prt in seg['parts']:
            if prt['shape'] == 'mass':
                continue
            px, py, pz = prt['pos']
            bad = bool(report[name]['probs'][prt['name']])
            col = 'red' if bad else ('orange' if 'cell' in prt['name'] else 'tab:blue')
            if prt['shape'] == 'box':
                sx, sy, sz = prt['size']
                if abs(xc - px) <= sx / 2:
                    ax.add_patch(patches.Rectangle((py - sy / 2, pz - sz / 2), sy, sz, alpha=0.5, color=col))
                    ax.text(py, pz, prt['name'], fontsize=6, ha='center', va='center')
            elif prt['shape'] == 'cyl_x':
                d, ln = prt['size']
                if abs(xc - px) <= ln / 2:
                    ax.add_patch(patches.Circle((py, pz), d / 2, alpha=0.5, color=col))
                    ax.text(py, pz, prt['name'], fontsize=6, ha='center', va='center')
            else:
                d, ln = prt['size']
                if abs(xc - px) <= d / 2:
                    ax.add_patch(patches.Rectangle((py - d / 2, pz - ln / 2), d, ln, alpha=0.5, color=col))
        ax.set_aspect('equal')
        ax.set_xlim(-55, 55)
        ax.set_ylim(-55, 55)
        ax.set_title(ttl, fontsize=9)
        ax.invert_xaxis()   # 앞에서 본 모습 (왼쪽 = +y)
    fig.suptitle('two_real 부품 배치 — 검정 원 = 몸통 Ø56, 초록 원 = 배관 내경 Ø100.8, 회색 = 바퀴 모듈 공간', fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--layout', default='layout.yaml')
    ap.add_argument('--plot', default='', help='배치도 PNG 경로')
    a = ap.parse_args()
    mdir = os.path.join(SRC_MODELS, a.model)
    layout = yaml.safe_load(open(os.path.join(mdir, a.layout), encoding='utf-8'))
    rp = yaml.safe_load(open(os.path.join(mdir, 'robot.yaml'), encoding='utf-8'))['/**']['ros__parameters']
    report = check(layout)
    ok_all = True
    for seg_name, r in report.items():
        print(f'\n[{seg_name}] 길이 {r["length"]} mm, 질량 {r["mass"]:.0f} g (몸통 {r["hull"]:.0f} g), '
              f'무게중심 ({r["com"][0]:.1f}, {r["com"][1]:.1f}, {r["com"][2]:.1f}) mm')
        for name, pr in r['probs'].items():
            if pr:
                ok_all = False
                print(f'  ✗ {name}: ' + ', '.join(sorted(set(pr))))
        if r['est']:
            print('  (추정 치수: ' + ', '.join(r['est']) + ')')
    total = sum(r['mass'] for r in report.values())
    print(f'\n총 질량 {total:.0f} g  ->  {"배치 OK" if ok_all else "배치 문제 있음 (빨강으로 표시)"}')
    if a.layout == 'layout.yaml':
        write_xacro(layout, report, rp, os.path.join(mdir, 'components.urdf.xacro'))
    if a.plot:
        plot(layout, report, rp, a.plot)
        print(f'배치도: {a.plot}')
    if a.apply:
        apply_params(os.path.join(mdir, 'robot.yaml'), report, rp)
        print('robot.yaml 갱신 (seg_len, seg_mass, seg_com, spawn_x)')


if __name__ == '__main__':
    main()
