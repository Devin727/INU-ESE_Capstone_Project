import os
from glob import glob
from setuptools import setup

package_name = 'pipe_robot_sim'


def tree(src):
    """Install every file under src/ keeping the folder layout (models/<name>/...)."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='황제펭귄살리기프로젝트',
    description='100A PVC 배관 검사 로봇 시뮬레이션',
    license='MIT',
    entry_points={
        'console_scripts': [
            'generate_world = pipe_robot_sim.generate_world:main',
            'encoder_sim = pipe_robot_sim.encoder_sim:main',
            'motor_driver = pipe_robot_sim.motor_driver:main',
            'odometry = pipe_robot_sim.odometry:main',
            'pipe_markers = pipe_robot_sim.pipe_markers:main',
            'joint_follow = pipe_robot_sim.joint_follow:main',
            'clamp_torque = pipe_robot_sim.clamp_torque:main',
            'oracle_steer = pipe_robot_sim.oracle_steer:main',
            'bench = pipe_robot_sim.bench:main',
            'layout_check = pipe_robot_sim.layout_check:main',
        ],
    },
)
