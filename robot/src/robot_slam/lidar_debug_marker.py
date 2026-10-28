import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

FRAME_ID = "base_link"

ROBOT_WIDTH = 0.30
ROBOT_LENGTH = 0.40

FRONT_ANGLE = 50
SIDE_MIN = 50
SIDE_MAX = 135

RADIUS = 1.0


class LidarDebugMarker(Node):
    def __init__(self):
        super().__init__("lidar_debug_marker")
        self.pub = self.create_publisher(MarkerArray, "/lidar_debug_markers_array", 10)
        self.timer = self.create_timer(0.2, self.publish_markers)

    def make_marker(self, marker_id, marker_type, ns):
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        return marker

    def publish_markers(self):
        arr = MarkerArray()

        arr.markers.append(self.robot_box_marker())
        arr.markers.append(self.sector_marker(1, "front", -FRONT_ANGLE, FRONT_ANGLE, 0.0, 1.0, 0.0))
        arr.markers.append(self.sector_marker(2, "left", -SIDE_MAX, -SIDE_MIN, 0.0, 0.2, 1.0))
        arr.markers.append(self.sector_marker(3, "right", SIDE_MIN, SIDE_MAX, 1.0, 1.0, 0.0))

        self.pub.publish(arr)

    def robot_box_marker(self):
        marker = self.make_marker(0, Marker.LINE_STRIP, "robot_box")
        marker.scale.x = 0.03
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        half_w = ROBOT_WIDTH / 2.0
        front = ROBOT_LENGTH / 2.0
        rear = -ROBOT_LENGTH / 2.0

        for x, y in [
            (front, half_w),
            (front, -half_w),
            (rear, -half_w),
            (rear, half_w),
            (front, half_w),
        ]:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.03
            marker.points.append(p)

        return marker

    def sector_marker(self, marker_id, ns, start_deg, end_deg, r, g, b):
        marker = self.make_marker(marker_id, Marker.TRIANGLE_LIST, ns)

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.65

        step = 3

        for deg in range(int(start_deg), int(end_deg), step):
            p0 = Point()
            p0.x = 0.0
            p0.y = 0.0
            p0.z = 0.01

            p1 = Point()
            p1.x = RADIUS * math.cos(math.radians(deg))
            p1.y = -RADIUS * math.sin(math.radians(deg))
            p1.z = 0.01

            p2 = Point()
            p2.x = RADIUS * math.cos(math.radians(deg + step))
            p2.y = -RADIUS * math.sin(math.radians(deg + step))
            p2.z = 0.01

            # 정방향 삼각형
            marker.points.append(p0)
            marker.points.append(p1)
            marker.points.append(p2)

            # 역방향 삼각형 (반대쪽에서 봐도 밝게 보이도록)
            marker.points.append(p0)
            marker.points.append(p2)
            marker.points.append(p1)

        return marker


def main():
    rclpy.init()
    node = LidarDebugMarker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
