"""코스 이름 -> Gazebo 월드 SDF 파일 생성.

예) ros2 run pipe_robot_sim generate_world elbow90 -o /tmp/elbow90.sdf --fitting90 ts90
"""
import argparse
import os

from ament_index_python.packages import get_package_share_directory

from pipe_robot_sim.pipe_geometry import build_course, load_config, world_sdf, write_collision_stl, write_visual_stl


def overrides(fitting90='', fitting45=''):
    """코스 안의 90°/45° 엘보를 다른 프리셋으로 일괄 교체."""
    out = {}
    if fitting90:
        out.update({'dv90': fitting90, 'dv_ll90': fitting90, 'ts90': fitting90})
    if fitting45:
        out.update({'dv45': fitting45, 'ts45': fitting45})
    return out


def default_courses_file():
    return os.path.join(get_package_share_directory('pipe_robot_sim'), 'config', 'courses.yaml')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('course')
    ap.add_argument('-o', '--out', default='')
    ap.add_argument('--courses', default='')
    ap.add_argument('--fitting90', default='')
    ap.add_argument('--fitting45', default='')
    ap.add_argument('--no-cutaway', action='store_true')
    a = ap.parse_args()

    cfg = load_config(a.courses or default_courses_file())
    b = build_course(cfg, a.course, overrides(a.fitting90, a.fitting45))
    out = a.out or f'{a.course}.sdf'
    mesh = os.path.abspath(os.path.splitext(out)[0] + '_pipe.stl')
    cmesh = os.path.abspath(os.path.splitext(out)[0] + '_pipe_collision.stl')
    write_visual_stl(b, mesh, cutaway=not a.no_cutaway)
    write_collision_stl(b, cmesh)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(world_sdf(b, cutaway=not a.no_cutaway, visual_mesh=mesh, collision_mesh=cmesh))
    print(f'{out}: 박스 {len(b.staves)}개, 중심선 길이 {b.s:.3f} m, 특징점 {b.features}')


if __name__ == '__main__':
    main()
