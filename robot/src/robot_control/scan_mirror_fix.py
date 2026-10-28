#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class ScanMirrorFix(Node):

    def __init__(self):
        super().__init__("scan_mirror_fix")

        # 원본 YDLIDAR 스캔
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

        # 앞뒤 반전이 수정된 스캔
        self.publisher = self.create_publisher(
            LaserScan,
            "/scan_fixed",
            qos_profile_sensor_data
        )

        self.get_logger().info(
            "LaserScan mirror correction started: /scan -> /scan_fixed"
        )

    @staticmethod
    def normalize_angle(angle):
        """
        각도를 -pi ~ +pi 범위로 정규화
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def scan_callback(self, msg):
        count = len(msg.ranges)

        if count == 0:
            return

        corrected_ranges = [float("inf")] * count

        if len(msg.intensities) == count:
            corrected_intensities = [0.0] * count
        else:
            corrected_intensities = []

        for source_index, distance in enumerate(msg.ranges):

            # 원본 포인트 각도
            source_angle = (
                msg.angle_min
                + source_index * msg.angle_increment
            )

            # X축만 반전
            corrected_angle = math.pi - source_angle
            corrected_angle = self.normalize_angle(corrected_angle)

            # 보정된 각도를 출력 배열 인덱스로 변환
            corrected_index = round(
                (corrected_angle - msg.angle_min)
                / msg.angle_increment
            )

            # 360도 스캔 배열 범위 보정
            corrected_index %= count

            corrected_ranges[corrected_index] = distance

            if corrected_intensities:
                corrected_intensities[corrected_index] = (
                    msg.intensities[source_index]
                )

        output = LaserScan()

        output.header = msg.header

        output.angle_min = msg.angle_min
        output.angle_max = msg.angle_max
        output.angle_increment = msg.angle_increment

        output.time_increment = msg.time_increment
        output.scan_time = msg.scan_time

        output.range_min = msg.range_min
        output.range_max = msg.range_max

        output.ranges = corrected_ranges
        output.intensities = corrected_intensities

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = ScanMirrorFix()

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
