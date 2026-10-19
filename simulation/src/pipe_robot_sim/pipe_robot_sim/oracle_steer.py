"""좌우 속도 차 조향 (무한궤도용), oracle: 시뮬이 아는 배관 중심선으로 몸체를 앞쪽 배관 방향으로 돌림.
실제 로봇에서는 전방 ToF·카메라로 엘보 방향을 추정해야 함.

sub  /ground_truth/odom
pub  /steer_cmd (std_msgs/Float64, rad/s) -> motor_driver 가 좌우 바퀴 속도에 더함
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from pipe_robot_sim.generate_world import default_courses_file, overrides
from pipe_robot_sim.joint_follow import quat_to_rot
from pipe_robot_sim.odometry import q_from_msg
from pipe_robot_sim.pipe_geometry import build_course, load_config, nearest_tangent


class OracleSteer(Node):
    def __init__(self):
        super().__init__('oracle_steer')
        d = self.declare_parameter
        self.look = d('steer_lookahead', 0.08).value
        self.k = d('steer_gain', 3.0).value
        self.wmax = d('steer_max', 1.5).value
        course = d('course', 'straight').value
        path = d('courses_file', '').value or default_courses_file()
        self.course = build_course(load_config(path), course,
                                   overrides(d('fitting90', '').value, d('fitting45', '').value))
        self.pub = self.create_publisher(Float64, 'steer_cmd', 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_gt, 10)

    def on_gt(self, msg):
        pos = msg.pose.pose.position
        R = quat_to_rot(q_from_msg(msg.pose.pose.orientation))
        p = np.array([pos.x, pos.y, pos.z]) + R[:, 0] * self.look
        t = nearest_tangent(self.course, p)
        tb = R.T @ t                       # 목표 방향을 몸체 좌표로
        err = math.atan2(tb[1], tb[0])     # 왼쪽(+) 으로 돌아야 하는 각도
        self.pub.publish(Float64(data=max(-self.wmax, min(self.wmax, self.k * err))))


def main():
    rclpy.init()
    rclpy.spin(OracleSteer())


if __name__ == '__main__':
    main()
