"""모터 드라이버 + DC 모터 시뮬 (ESP32 + DRV8833 역할).

/cmd_vel(linear.x, m/s) -> 바퀴별 목표 각속도 -> 엔코더 카운트로 속도 추정 -> PID -> PWM duty(-1~1)
-> 모터 토크 = duty x 스톨토크 x (전원전압/정격전압) -> Gazebo 바퀴 관절.
역기전력(속도에 비례해 줄어드는 토크)은 URDF 관절 damping(= 스톨토크/무부하속도)으로 물리엔진이 처리.
=> 바퀴 토크 = duty·τ_stall·V/V_rated − (τ_stall/ω_nl)·ω  (일반적인 DC 모터 토크-속도 직선)

sub  /cmd_vel (geometry_msgs/Twist), /wheel_encoders (std_msgs/Int64MultiArray)
pub  /motor/<joint>/torque (std_msgs/Float64, 브리지 -> Gazebo)
     /motor/duty, /motor/wheel_speed (std_msgs/Float64MultiArray, 디버그)
drive_mode: speed_pid (바퀴마다 속도 PID) | open_loop (duty = 목표/최고속도, 피드백 없음)
            | diff (평균 속도만 PID, 모든 바퀴 같은 토크 = 전자식 차동)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int64MultiArray


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        d = self.declare_parameter
        self.joints = list(d('wheel_joints', ['']).value)
        self.r = d('wheel_radius', 0.012).value
        self.cpr = d('encoder_cpr', 2800).value
        self.tau_stall = d('motor_stall_torque', 0.147).value
        self.w_nl = d('motor_no_load_speed', 16.8).value
        self.v_ratio = d('supply_voltage', 7.4).value / d('rated_voltage', 6.0).value
        self.rate = d('control_rate', 100.0).value
        self.mode = d('drive_mode', 'speed_pid').value
        self.kp, self.ki, self.kd = d("kp", 0.01).value, d("ki", 1.0).value, d('kd', 0.0).value
        self.cmd_timeout = d('cmd_timeout', 0.5).value
        # 좌우 속도 차 조향 (무한궤도 등): 바퀴별 +1 = 왼쪽으로 돌 때 빨라지는 쪽(오른쪽) / -1 반대 / 0 조향 없음
        self.sides = list(d('wheel_sides', [0.0]).value)
        self.sep = d('track_separation', 0.0).value
        self.steer = 0.0                  # rad/s (angular.z + /steer_cmd)
        self.steer_extra = 0.0

        n = len(self.joints)
        self.target = 0.0                 # rad/s (모든 바퀴 동일)
        self.last_cmd = None
        self.counts = None
        self.prev_counts = None
        self.speed = [0.0] * n
        self.integ = [0.0] * n
        self.prev_err = [0.0] * n

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Float64, 'steer_cmd', lambda m: setattr(self, 'steer_extra', m.data), 10)
        self.create_subscription(Int64MultiArray, 'wheel_encoders', self.on_enc, 10)
        self.torque_pubs = [self.create_publisher(Float64, f'motor/{j}/torque', 10) for j in self.joints]
        self.duty_pub = self.create_publisher(Float64MultiArray, 'motor/duty', 10)
        self.speed_pub = self.create_publisher(Float64MultiArray, 'motor/wheel_speed', 10)
        self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info(
            f'mode={self.mode}, 최고속도 {self.w_nl * self.v_ratio * self.r * 100:.1f} cm/s (무부하)')

    def on_cmd(self, msg):
        self.target = msg.linear.x / self.r
        self.steer = msg.angular.z
        self.last_cmd = self.get_clock().now()

    def on_enc(self, msg):
        self.counts = list(msg.data)

    def step(self):
        dt = 1.0 / self.rate
        if self.counts is None:
            return
        if self.prev_counts is not None:
            self.speed = [(c - p) * 2 * math.pi / self.cpr / dt for c, p in zip(self.counts, self.prev_counts)]
        self.prev_counts = list(self.counts)

        target = self.target
        if self.last_cmd is None or (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            target = 0.0    # 명령 끊기면 정지 (펌웨어 타임아웃과 같은 역할)

        w_max = self.w_nl * self.v_ratio
        duties = []
        if self.mode == 'diff':
            # 전자식 차동: 평균 속도 하나만 PID, 모든 바퀴에 같은 duty (= 같은 토크)
            # -> 엘보에서 바깥·안쪽 바퀴가 서로 다른 속도로 자유롭게 돎 (바퀴끼리 싸우지 않음)
            mean = sum(self.speed) / len(self.speed)
            err = target - mean
            self.integ[0] += err * (1.0 / self.rate)
            u = target / w_max + self.kp * err + self.ki * self.integ[0]
            if abs(u) > 1.0:
                self.integ[0] -= err * (1.0 / self.rate)
            if target == 0.0 and abs(mean) < 0.05:
                self.integ[0] = 0.0
            u = max(-1.0, min(1.0, u))
            for i in range(len(self.joints)):
                duties.append(u)
                self.torque_pubs[i].publish(Float64(data=u * self.tau_stall * self.v_ratio))
            self.duty_pub.publish(Float64MultiArray(data=duties))
            self.speed_pub.publish(Float64MultiArray(data=self.speed))
            return
        for i in range(len(self.joints)):
            side = self.sides[i] if i < len(self.sides) else 0.0
            tgt = target
            if target != 0.0 and side:
                tgt += side * (self.steer + self.steer_extra) * self.sep / 2 / self.r
            ff = tgt / w_max
            if self.mode == 'open_loop':
                u = ff
            else:
                err = tgt - self.speed[i]
                self.integ[i] += err * dt
                deriv = (err - self.prev_err[i]) / dt
                self.prev_err[i] = err
                u = ff + self.kp * err + self.ki * self.integ[i] + self.kd * deriv
                if abs(u) > 1.0:                       # 포화 시 적분 되돌림 (windup 방지)
                    self.integ[i] -= err * dt
                if tgt == 0.0 and abs(self.speed[i]) < 0.05:
                    self.integ[i] = 0.0
            u = max(-1.0, min(1.0, u))
            duties.append(u)
            self.torque_pubs[i].publish(Float64(data=u * self.tau_stall * self.v_ratio))

        self.duty_pub.publish(Float64MultiArray(data=duties))
        self.speed_pub.publish(Float64MultiArray(data=self.speed))


def main():
    rclpy.init()
    rclpy.spin(MotorDriver())


if __name__ == '__main__':
    main()
