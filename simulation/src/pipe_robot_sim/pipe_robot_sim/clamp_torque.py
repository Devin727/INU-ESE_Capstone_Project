"""클램프 서보 토크 (pirate_v 등): clamp_joints 에 clamp_torque [N·m] 를 계속 걸어 V를 벌려 벽을 누름.

pub  /motor/<joint>/torque (std_msgs/Float64) -> 브리지 -> Gazebo ApplyJointForce
sub  /clamp_scale (std_msgs/Float64, 0~1, 선택) -> 토크 배율 (엘보 통과 시 힘 빼기 실험용)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ClampTorque(Node):
    def __init__(self):
        super().__init__('clamp_torque')
        self.joints = list(self.declare_parameter('clamp_joints', ['']).value)
        self.torque = list(self.declare_parameter('clamp_torque', [0.0]).value)
        self.scale = 1.0
        self.pubs = [self.create_publisher(Float64, f'motor/{j}/torque', 10) for j in self.joints]
        self.create_subscription(Float64, 'clamp_scale', lambda m: setattr(self, 'scale', m.data), 10)
        self.create_timer(0.02, self.step)

    def step(self):
        for pub, t in zip(self.pubs, self.torque):
            pub.publish(Float64(data=t * self.scale))


def main():
    rclpy.init()
    rclpy.spin(ClampTorque())


if __name__ == '__main__':
    main()
