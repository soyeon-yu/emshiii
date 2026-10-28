import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LidarAngleCheck(Node):
    def __init__(self):
        super().__init__("lidar_angle_check")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos
        )

        self.get_logger().info(
            "Place one object close to the LiDAR and check the minimum-distance angle."
        )

    def scan_callback(self, msg):
        closest_distance = float("inf")
        closest_angle = None

        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            if distance < closest_distance:
                closest_distance = distance
                closest_angle = angle_deg

        if closest_angle is not None:
            print(
                f"Closest distance: {closest_distance:.2f} m | "
                f"Angle: {closest_angle:.1f} deg"
            )


def main():
    rclpy.init()
    node = LidarAngleCheck()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
