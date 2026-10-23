"""배관 로봇 시뮬 실행.

ros2 launch pipe_robot_sim sim.launch.py course:=elbow90 model:=<모델>
  course     : config/courses.yaml 의 코스 이름 (straight | elbow45 | elbow90 | demo | ... )
  model      : models/<model>/<model>.urdf.xacro + robot.yaml. 비우면 배관만 띄움
  fitting90  : 코스의 90° 엘보를 다른 프리셋으로 교체 (dv90 | dv_ll90 | ts90)
  fitting45  : 코스의 45° 엘보 교체 (dv45 | ts45)
  suspension : xacro 로 넘기는 바퀴 스프링 구성 (모델이 쓰는 경우만)
  gui, rviz  : true | false
  rviz_config: config/ 안의 RViz 설정 파일 (기본 sim.rviz, 평면 엘보는 sim_top.rviz)
"""
import math
import os
import re

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pipe_robot_sim.generate_world import overrides
from pipe_robot_sim.pipe_geometry import build_course, load_config, world_sdf, write_collision_stl, write_visual_stl

PKG = 'pipe_robot_sim'
ROBOT = 'pipe_robot'
TMP = f'/tmp/pipe_robot_sim/{os.getpid()}'   # 실행마다 따로 (동시에 여러 시뮬을 돌려도 파일이 안 섞임)


def bridge_yaml(wheel_joints, servo_joints=(), servo_modes=()):
    def ent(ros, gz, ros_t, gz_t, direction):
        return dict(ros_topic_name=ros, gz_topic_name=gz, ros_type_name=ros_t,
                    gz_type_name=gz_t, direction=direction)
    out = [
        ent('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'ignition.msgs.Clock', 'GZ_TO_ROS'),
        ent('/joint_states', '/joint_states', 'sensor_msgs/msg/JointState', 'ignition.msgs.Model', 'GZ_TO_ROS'),
        ent('/imu', '/imu', 'sensor_msgs/msg/Imu', 'ignition.msgs.IMU', 'GZ_TO_ROS'),
        ent('/ground_truth/odom', '/ground_truth/odom', 'nav_msgs/msg/Odometry', 'ignition.msgs.Odometry', 'GZ_TO_ROS'),
        # 링크별 정답 자세 (bench 평가용: 마디가 배관과 일자인지)
        ent('/gz_poses', '/world/pipe_world/dynamic_pose/info', 'tf2_msgs/msg/TFMessage', 'ignition.msgs.Pose_V', 'GZ_TO_ROS'),
    ]
    for t in ('tof/front', 'tof/left', 'tof/right'):   # ToF (모델에 없으면 그냥 안 들어옴)
        out.append(ent(f'/{t}', f'/{t}', 'sensor_msgs/msg/LaserScan', 'ignition.msgs.LaserScan', 'GZ_TO_ROS'))
    # 전방 8x8 전체 (LaserScan 변환은 가운데 한 줄만 남김)
    out.append(ent('/tof/front/points', '/tof/front/points', 'sensor_msgs/msg/PointCloud2',
                   'ignition.msgs.PointCloudPacked', 'GZ_TO_ROS'))
    for j in wheel_joints:
        out.append(ent(f'/motor/{j}/torque', f'/model/{ROBOT}/joint/{j}/cmd_force',
                       'std_msgs/msg/Float64', 'ignition.msgs.Double', 'ROS_TO_GZ'))
    for j, mode in zip(servo_joints, servo_modes):      # 능동 서보 각도 명령 [rad]
        if mode == 'active':
            out.append(ent(f'/servo/{j}', f'/servo/{j}', 'std_msgs/msg/Float64', 'ignition.msgs.Double', 'ROS_TO_GZ'))
    return out


def model_dir(model):
    return os.path.join(get_package_share_directory(PKG), 'models', model)


def available_models():
    root = os.path.join(get_package_share_directory(PKG), 'models')
    if not os.path.isdir(root):
        return []
    return sorted(m for m in os.listdir(root) if os.path.isfile(os.path.join(root, m, 'robot.yaml')))


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    share = get_package_share_directory(PKG)
    course, model = arg('course'), arg('model')
    gui, rviz = arg('gui') == 'true', arg('rviz') == 'true'
    fit = overrides(arg('fitting90'), arg('fitting45'))
    os.makedirs(TMP, exist_ok=True)

    actions = []
    rp = None
    if model:
        params_file = os.path.join(model_dir(model), 'robot.yaml')
        if not os.path.isfile(params_file):
            raise RuntimeError(f"모델 '{model}' 없음 (models/{model}/robot.yaml). 있는 모델: {available_models() or '없음'}")
        with open(params_file, encoding='utf-8') as f:
            rp = yaml.safe_load(f)['/**']['ros__parameters']
        # PIPE_PARAMS='k=v,k=v': robot.yaml 값 덮어쓰기 (비교 시험용, 임시 파일로 노드에 넘김)
        extra = {k: yaml.safe_load(v) for k, v in (kv.split('=', 1) for kv in os.environ.get('PIPE_PARAMS', '').split(',') if '=' in kv)}
        if extra:
            rp.update(extra)
            params_file = os.path.join(TMP, 'robot_params.yaml')
            with open(params_file, 'w', encoding='utf-8') as f:
                yaml.safe_dump({'/**': {'ros__parameters': rp}}, f, allow_unicode=True)
    else:
        actions.append(LogInfo(msg=f'model 인자가 비어 있어 배관만 띄웁니다. 있는 모델: {available_models() or "없음"}'))

    # 1) 배관 월드
    courses_file = os.path.join(share, 'config', 'courses.yaml')
    cfg = load_config(courses_file)
    course_geo = build_course(cfg, course, fit)
    world = os.path.join(TMP, f'{course}.sdf')
    mesh = os.path.join(TMP, f'{course}_pipe.stl')
    cmesh = os.path.join(TMP, f'{course}_pipe_collision.stl')
    write_visual_stl(course_geo, mesh)
    write_collision_stl(course_geo, cmesh)
    smesh = None
    if rp and rp.get('tof'):   # ToF 센서가 있는 모델: 센서 전용 완전한 배관 + 렌더링 센서 시스템
        smesh = os.path.join(TMP, f'{course}_pipe_sensor.stl')
        write_visual_stl(course_geo, smesh, cutaway=False)
    with open(world, 'w', encoding='utf-8') as f:
        # 물리 설정 (robot.yaml): dart_collision_detector (ode 기본 | bullet | fcl | dart), pipe_box_collision (배관 충돌체를
        # 메시 대신 박스 조각으로, 느림). 환경변수 PIPE_PHYSICS_EXTRA / PIPE_BOX_COLLISION 로도 줄 수 있음 (시험용)
        cd = os.environ.get('PIPE_DART_DETECTOR') or (rp or {}).get('dart_collision_detector')   # 환경변수가 우선 (시험용)
        if cd:
            os.environ['PIPE_PHYSICS_EXTRA'] = f'<dart><collision_detector>{cd}</collision_detector></dart>'
        box_col = bool((rp or {}).get('pipe_box_collision') or os.environ.get('PIPE_BOX_COLLISION'))
        f.write(world_sdf(course_geo, visual_mesh=mesh, collision_mesh=None if box_col else cmesh, sensor_mesh=smesh))

    sim = {'use_sim_time': True}
    gz_cmd = ['ign', 'gazebo', '-r', world] if gui else ['ign', 'gazebo', '-s', '-r', world]
    if gui and arg('fullscreen') == 'true':   # 화면 크기 창 (기본 GUI 설정을 복사해 창 크기만 바꿈)
        base = os.path.expanduser('~/.ignition/gazebo/6/gui.config')
        if os.path.exists(base):
            g = open(base, encoding='utf-8').read()
            g = re.sub(r'<width>\d+</width>', '<width>1920</width>', g)
            g = re.sub(r'<height>\d+</height>', '<height>1050</height>', g)
            g = g.replace('<window>', '<window>\n  <position_x>0</position_x>\n  <position_y>0</position_y>', 1)
            g = g.replace('<dialog name="quick_start" show_again="true"/>', '<dialog name="quick_start" show_again="false"/>')
            gcfg = os.path.join(TMP, 'gui_fullscreen.config')
            open(gcfg, 'w', encoding='utf-8').write(g)
            gz_cmd = ['ign', 'gazebo', '-r', '--gui-config', gcfg, world]
    actions += [
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        Node(package=PKG, executable='pipe_markers', output='screen',
             parameters=[sim, {'course': course, 'courses_file': courses_file,
                               'fitting90': arg('fitting90'), 'fitting45': arg('fitting45')}]),
    ]

    # 2) 로봇 (모델이 있을 때만)
    if rp is not None:
        urdf = xacro.process_file(os.path.join(model_dir(model), f'{model}.urdf.xacro'), mappings={
            'params_file': params_file,
            'pipe_inner_d': str(cfg['pipe']['inner_d']),
            'suspension': arg('suspension'),
            # PIPE_XACRO_ARGS='k=v,k=v': 모델 xacro 인자 덮어쓰기 (비교 시험용)
            **dict(kv.split('=', 1) for kv in os.environ.get('PIPE_XACRO_ARGS', '').split(',') if '=' in kv),
        }).toxml()
        bridge_file = os.path.join(TMP, 'bridge.yaml')
        with open(bridge_file, 'w', encoding='utf-8') as f:
            yaml.safe_dump(bridge_yaml(rp.get('wheel_joints', []) + rp.get('roll_joints', []),   # 굴림 바퀴도 토크 입력
                                       rp.get('servo_joints', []), rp.get('servo_modes', [])), f)
        spawn_x = rp['spawn_x'] if 'spawn_x' in rp else 0.03 + rp['body_len'] / 2 + 0.03   # 입구에서 약간 안쪽
        # spawn_roll_deg: 배관 축 둘레로 돌린 채 시작 (예: 뱀형 V자를 PIRATE처럼 V 수평으로)
        # spawn_dz: 로봇 기준 위(+)/아래(-) 보정 -> 롤만큼 돌려 world 의 y, z 로
        roll = math.radians(rp.get('spawn_roll_deg', 0.0))
        dz = rp.get('spawn_dz', 0.0)
        spawn_y, spawn_z = -math.sin(roll) * dz, course_geo.path[0][0][2] + math.cos(roll) * dz
        actions += [
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[sim, {'robot_description': urdf}]),
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', ROBOT, '-topic', 'robot_description',
                            '-x', str(spawn_x), '-y', str(spawn_y), '-z', str(spawn_z), '-R', str(roll)]),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 parameters=[sim, {'config_file': bridge_file}]),
            Node(package=PKG, executable='odometry', output='screen',
                 parameters=[params_file, sim, {'imu_mode': arg('imu_mode')}]),
        ]
        if rp.get('wheel_joints'):   # 바퀴 모터·엔코더
            actions += [Node(package=PKG, executable='encoder_sim', parameters=[params_file, sim]),
                        Node(package=PKG, executable='motor_driver', output='screen', parameters=[params_file, sim])]
        if rp.get('snake_gait'):   # 뱀형 V자 관절 제어
            actions.append(Node(package=PKG, executable='snake_gait', output='screen', parameters=[
                params_file, sim, {'pipe_inner_d': cfg['pipe']['inner_d'], 'course': course, 'courses_file': courses_file,
                                   'fitting90': arg('fitting90'), 'fitting45': arg('fitting45')}]))
        if rp.get('airo_ctrl'):   # AIRo-7.1 제어 (가운데 관절 누름, 굴림 바퀴로 굽힘 평면)
            actions.append(Node(package=PKG, executable='airo_ctrl', output='screen', parameters=[
                params_file, sim, {'course': course, 'courses_file': courses_file,
                                   'fitting90': arg('fitting90'), 'fitting45': arg('fitting45')}]))
        if gui:   # Gazebo 카메라가 로봇을 비스듬히 위에서 따라감 (GUI 오른쪽 위 메뉴에서 해제 가능)
            svc = lambda srv, typ, req: ExecuteProcess(cmd=[  # noqa: E731
                'ign', 'service', '-s', srv, '--reqtype', typ, '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '5000', '--req', req])
            for t in (15.0, 30.0):   # GUI 뜨는 시간이 들쭉날쭉해서 두 번 보냄
                actions.append(TimerAction(period=t, actions=[
                    svc('/gui/follow/offset', 'ignition.msgs.Vector3d', 'x: -0.3, y: -0.15, z: 0.5'),
                    svc('/gui/follow', 'ignition.msgs.StringMsg', f'data: "{ROBOT}"')]))
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[sim],
                            arguments=['-d', os.path.join(share, 'config', arg('rviz_config'))]))
    return actions


def generate_launch_description():
    args = [
        ('course', 'straight'), ('model', ''), ('fitting90', ''), ('fitting45', ''),
        ('suspension', 'all'), ('imu_mode', 'gyro'), ('gui', 'true'), ('rviz', 'true'), ('fullscreen', 'false'),
        ('rviz_config', 'sim.rviz'),   # config/ 안의 RViz 설정 (sim_top.rviz = 위에서 내려다봄)
    ]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=v) for n, v in args]
                             + [OpaqueFunction(function=setup)])
