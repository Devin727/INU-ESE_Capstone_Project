"""RViz용 배관 표시: Gazebo와 같은 박스 조각의 안쪽 면을 TRIANGLE_LIST로 발행 (위쪽은 잘라서 안이 보이게)."""
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from pipe_robot_sim.generate_world import default_courses_file, overrides
from pipe_robot_sim.pipe_geometry import build_course, inner_triangles, load_config


class PipeMarkers(Node):
    def __init__(self):
        super().__init__('pipe_markers')
        d = self.declare_parameter
        course = d('course', 'straight').value
        path = d('courses_file', '').value or default_courses_file()
        b = build_course(load_config(path), course, overrides(d('fitting90', '').value, d('fitting45', '').value))

        m = Marker()
        m.header.frame_id = 'world'
        m.ns, m.id, m.type, m.action = 'pipe', 0, Marker.TRIANGLE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.85, 0.8, 0.45
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in inner_triangles(b)]

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(MarkerArray, 'pipe_markers', qos).publish(MarkerArray(markers=[m]))
        self.get_logger().info(f'{course}: 중심선 {b.s:.2f} m, 특징점 {b.features}')


def main():
    rclpy.init()
    rclpy.spin(PipeMarkers())


if __name__ == '__main__':
    main()
