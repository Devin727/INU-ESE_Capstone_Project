"""엔코더 시뮬: Gazebo 바퀴 관절 각도 -> 정수 카운트 (실제 쿼드러처 엔코더처럼 양자화).

sub  /joint_states            (sensor_msgs/JointState, Gazebo 정답 각도)
pub  /wheel_encoders          (std_msgs/Int64MultiArray, wheel_joints 순서의 누적 카운트)
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray


class EncoderSim(Node):
    def __init__(self):
        super().__init__('encoder_sim')
        self.joints = list(self.declare_parameter('wheel_joints', ['']).value)
        self.cpr = self.declare_parameter('encoder_cpr', 2800).value
        rate = self.declare_parameter('encoder_rate', 200.0).value

        self.angle = {j: None for j in self.joints}
        self.create_subscription(JointState, 'joint_states', self.on_js, 10)
        self.pub = self.create_publisher(Int64MultiArray, 'wheel_encoders', 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_js(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.angle:
                self.angle[name] = pos

    def tick(self):
        if any(a is None for a in self.angle.values()):
            return
        counts = [math.floor(self.angle[j] / (2 * math.pi) * self.cpr) for j in self.joints]
        self.pub.publish(Int64MultiArray(data=counts))


def main():
    rclpy.init()
    rclpy.spin(EncoderSim())


if __name__ == '__main__':
    main()
