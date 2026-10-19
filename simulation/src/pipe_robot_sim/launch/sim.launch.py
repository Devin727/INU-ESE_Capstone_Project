"""배관 로봇 시뮬 실행.

ros2 launch pipe_robot_sim sim.launch.py course:=elbow90 model:=three_wheel
  course     : straight | elbow45 | elbow90 | demo   (config/courses.yaml)
  model      : models/<model>/<model>.urdf.xacro + robot.yaml
  fitting90  : 코스의 90° 엘보를 다른 프리셋으로 교체 (dv90 | dv_ll90 | ts90)
  fitting45  : 코스의 45° 엘보 교체 (dv45 | ts45)
  suspension : all | top
  gui, rviz  : true | false
"""
import os
import re

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
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


def rp_peek(model):
    with open(os.path.join(get_package_share_directory(PKG), 'models', model, 'robot.yaml'), encoding='utf-8') as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    share = get_package_share_directory(PKG)
    course, model = arg('course'), arg('model')
    gui, rviz = arg('gui') == 'true', arg('rviz') == 'true'
    fit = overrides(arg('fitting90'), arg('fitting45'))
    os.makedirs(TMP, exist_ok=True)

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
    if rp_peek(model).get('tof'):   # ToF 센서가 있는 모델: 센서 전용 완전한 배관 + 렌더링 센서 시스템
        smesh = os.path.join(TMP, f'{course}_pipe_sensor.stl')
        write_visual_stl(course_geo, smesh, cutaway=False)
    with open(world, 'w', encoding='utf-8') as f:
        f.write(world_sdf(course_geo, visual_mesh=mesh, collision_mesh=cmesh, sensor_mesh=smesh))

    # 2) 로봇 모델
    model_dir = os.path.join(share, 'models', model)
    params_file = os.path.join(model_dir, 'robot.yaml')
    with open(params_file, encoding='utf-8') as f:
        rp = yaml.safe_load(f)['/**']['ros__parameters']
    urdf = xacro.process_file(os.path.join(model_dir, f'{model}.urdf.xacro'), mappings={
        'params_file': params_file,
        'pipe_inner_d': str(cfg['pipe']['inner_d']),
        'suspension': arg('suspension'),
    }).toxml()

    # 3) 브리지 설정
    bridge_file = os.path.join(TMP, 'bridge.yaml')
    with open(bridge_file, 'w', encoding='utf-8') as f:
        entries = bridge_yaml(rp.get('wheel_joints', []) + rp.get('clamp_joints', []),
                              rp.get('servo_joints', []), rp.get('servo_modes', []))
        if rp.get('tracked'):   # 무한궤도: cmd_vel을 Gazebo TrackedVehicle로 바로
            entries.append(dict(ros_topic_name='/cmd_vel', gz_topic_name=f'/model/{ROBOT}/cmd_vel',
                                ros_type_name='geometry_msgs/msg/Twist', gz_type_name='ignition.msgs.Twist',
                                direction='ROS_TO_GZ'))
        yaml.safe_dump(entries, f)

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
    spawn_x = rp['spawn_x'] if 'spawn_x' in rp else 0.03 + rp['body_len'] / 2 + 0.03   # 입구에서 약간 안쪽

    actions = [
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim, {'robot_description': urdf}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', ROBOT, '-topic', 'robot_description',
                        '-x', str(spawn_x), '-y', '0', '-z', str(course_geo.path[0][0][2])]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[sim, {'config_file': bridge_file}]),
        Node(package=PKG, executable='odometry', output='screen',
             parameters=[params_file, sim, {'imu_mode': arg('imu_mode')}]),
        Node(package=PKG, executable='pipe_markers', output='screen',
             parameters=[sim, {'course': course, 'courses_file': courses_file,
                               'fitting90': arg('fitting90'), 'fitting45': arg('fitting45')}]),
    ]
    if gui:   # Gazebo 카메라가 로봇을 비스듬히 위에서 따라감 (GUI 오른쪽 위 메뉴에서 해제 가능)
        svc = lambda srv, typ, req: ExecuteProcess(cmd=[  # noqa: E731
            'ign', 'service', '-s', srv, '--reqtype', typ, '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000', '--req', req])
        for t in (15.0, 30.0):   # GUI 뜨는 시간이 들쭉날쭉해서 두 번 보냄
            actions.append(TimerAction(period=t, actions=[
                svc('/gui/follow/offset', 'ignition.msgs.Vector3d', 'x: -0.3, y: -0.15, z: 0.5'),
                svc('/gui/follow', 'ignition.msgs.StringMsg', f'data: "{ROBOT}"')]))
    if not rp.get('tracked'):   # 바퀴 모터·엔코더 (무한궤도는 Gazebo가 직접 구동)
        actions += [Node(package=PKG, executable='encoder_sim', parameters=[params_file, sim]),
                    Node(package=PKG, executable='motor_driver', output='screen', parameters=[params_file, sim])]
    if rp.get('oracle_steer'):   # 좌우 속도 차 조향 (무한궤도)
        actions.append(Node(package=PKG, executable='oracle_steer', parameters=[
            params_file, sim, {'course': course, 'courses_file': courses_file,
                               'fitting90': arg('fitting90'), 'fitting45': arg('fitting45')}]))
    if rp.get('clamp_joints'):   # V 클램프 서보 토크 (pirate_v)
        actions.append(Node(package=PKG, executable='clamp_torque', parameters=[params_file, sim]))
    if 'active' in rp.get('servo_modes', []):   # 능동 관절 서보 제어
        actions.append(Node(package=PKG, executable='joint_follow', output='screen', parameters=[
            params_file, sim, {'course': course, 'courses_file': courses_file,
                               'fitting90': arg('fitting90'), 'fitting45': arg('fitting45')}]))
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[sim],
                            arguments=['-d', os.path.join(share, 'config', 'sim.rviz')]))
    return actions


def generate_launch_description():
    args = [
        ('course', 'straight'), ('model', 'three_wheel'), ('fitting90', ''), ('fitting45', ''),
        ('suspension', 'all'), ('imu_mode', 'gyro'), ('gui', 'true'), ('rviz', 'true'), ('fullscreen', 'false'),
    ]
    return LaunchDescription([DeclareLaunchArgument(n, default_value=v) for n, v in args]
                             + [OpaqueFunction(function=setup)])
