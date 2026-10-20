"""IMU 보조 제어 (센서만 사용, 배관 모양 정보 없음).

1) 롤 유지: 몸통이 배관 축 둘레로 넘어가지 않게 굴림 바퀴(roll_joints)에 토크.
   롤 추정 = 상보 필터: 가속도계가 보는 중력 방향(atan2(ay, az)) + 자이로 x 적분.
   몸통 축이 중력과 거의 나란하면(수직관) 가속도로는 롤을 못 구하므로 자이로만 적분.
   토크 = roll_sign x -(kp x (롤 - 목표) + kd x 롤속도), 크기는 roll_max_torque 로 제한.
2) 좌우 속도차: 자이로 z(몸통이 좌우로 도는 속도)를 /steer_cmd 로 보냄.
   motor_driver 가 wheel_sides, track_separation 으로 안쪽 바퀴는 느리게, 바깥 바퀴는 빠르게 돌림
   -> 엘보에서 좌우 바퀴가 같은 속도를 강요받아 서로 다투지(몸통을 비틀지) 않게.

sub  /imu (sensor_msgs/Imu)
pub  /motor/<roll_joint>/torque (std_msgs/Float64), /steer_cmd (std_msgs/Float64), /imu_assist/roll (디버그, deg)
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class ImuAssist(Node):
    def __init__(self):
        super().__init__('imu_assist')
        d = self.declare_parameter
        self.roll_joints = list(d('roll_joints', ['']).value)
        self.target = math.radians(d('roll_target_deg', 0.0).value)
        self.kp = d('roll_kp', 0.3).value
        self.kd = d('roll_kd', 0.02).value
        self.tmax = d('roll_max_torque', 0.15).value
        self.sign = d('roll_sign', 1.0).value
        self.alpha = d('roll_filter_alpha', 0.98).value      # 자이로 비중 (나머지 = 가속도)
        self.steer_gain = d('steer_gain', 1.0).value          # 0 이면 좌우 속도차 끔
        self.steer_tau = d('steer_filter_s', 0.05).value      # 자이로 z 저역 필터 시정수 [s]

        self.roll = None
        self.gz_f = 0.0
        self.prev_t = None
        self.torque_pubs = [self.create_publisher(Float64, f'motor/{j}/torque', 10) for j in self.roll_joints if j]
        self.steer_pub = self.create_publisher(Float64, 'steer_cmd', 10)
        self.roll_pub = self.create_publisher(Float64, 'imu_assist/roll', 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 50)
        self.get_logger().info(f'롤 유지 {self.roll_joints} (목표 {math.degrees(self.target):.0f}°), '
                               f'좌우 속도차 {"켬" if self.steer_gain else "끔"}')

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        a, w = msg.linear_acceleration, msg.angular_velocity
        if self.prev_t is None:
            self.prev_t = t
            self.roll = math.atan2(a.y, a.z)
            return
        dt = max(t - self.prev_t, 0.0)
        self.prev_t = t
        if dt == 0.0:
            return

        # 롤: 자이로 적분 + 중력 방향으로 보정 (중력이 몸통 y-z 평면에 충분히 있을 때만)
        self.roll += w.x * dt
        g_yz = math.hypot(a.y, a.z)
        if g_yz > 0.5 * 9.81:
            r_acc = math.atan2(a.y, a.z)
            err = math.atan2(math.sin(r_acc - self.roll), math.cos(r_acc - self.roll))   # ±180° 넘김 처리
            self.roll += (1.0 - self.alpha) * err
        self.roll = math.atan2(math.sin(self.roll), math.cos(self.roll))

        e = math.atan2(math.sin(self.roll - self.target), math.cos(self.roll - self.target))
        tau = -self.sign * (self.kp * e + self.kd * w.x)
        tau = max(-self.tmax, min(self.tmax, tau))
        for p in self.torque_pubs:
            p.publish(Float64(data=tau))
        self.roll_pub.publish(Float64(data=math.degrees(self.roll)))

        # 좌우 속도차: 몸통 z 축 회전 속도를 저역 필터해 그대로 조향 명령으로
        k = dt / (self.steer_tau + dt)
        self.gz_f += k * (w.z - self.gz_f)
        if self.steer_gain:
            self.steer_pub.publish(Float64(data=self.steer_gain * self.gz_f))


def main():
    rclpy.init()
    rclpy.spin(ImuAssist())


if __name__ == '__main__':
    main()
